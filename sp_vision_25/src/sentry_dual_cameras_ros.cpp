#include <fmt/core.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <list>
#include <memory>
#include <stdexcept>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <yaml-cpp/yaml.h>

#include "io/camera.hpp"
#include "io/ros2/gimbal_ros.hpp"
#include "src/async_detect_worker.hpp"

#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/omniperception/decider.hpp"

#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

// ==========================================
// 多线程取帧模型
// ==========================================
struct FrameData
{
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  std::uint64_t seq = 0;
};

class CameraStream
{
public:
  CameraStream(io::SNCamera *cam) : camera_(cam), running_(true)
  {
    thread_ = std::thread(&CameraStream::grab_loop, this);
  }

  ~CameraStream()
  {
    running_ = false;
    if (thread_.joinable())
      thread_.join();
  }

  bool get_latest(
      cv::Mat &out_img, std::chrono::steady_clock::time_point &out_t,
      std::uint64_t &last_seq)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (latest_frame_.img.empty() || latest_frame_.seq == last_seq)
      return false;

    // 【优化】使用 copyTo 替代 clone()。只要 out_img 在外部不被销毁，
    // copyTo 会自动复用内存，避免每帧分配数MB内存的巨大开销！
    latest_frame_.img.copyTo(out_img);
    out_t = latest_frame_.timestamp;
    last_seq = latest_frame_.seq;
    return true;
  }

private:
  void grab_loop()
  {
    while (running_)
    {
      cv::Mat img;
      auto t = std::chrono::steady_clock::now();
      camera_->read(img, t); // 持续抓取，清空底层缓存

      if (!img.empty())
      {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_frame_.img = img;
        latest_frame_.timestamp = t;
        ++latest_frame_.seq;
      }
    }
  }

  io::SNCamera *camera_;
  std::atomic<bool> running_;
  std::thread thread_;
  std::mutex mutex_;
  FrameData latest_frame_;
};
// ==========================================

const std::string keys =
    "{help h usage ? |                        | 输出命令行参数说明}"
    "{@config-path   | configs/demo.yaml      | 位置参数，yaml配置文件路径}";

int main(int argc, char *argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help"))
  {
    cli.printMessage();
    return 0;
  }

  auto config_path = cli.get<std::string>(0);
  const auto config = YAML::LoadFile(config_path);
  const auto imu_delay_ms = config["imu_delay_ms"]
      ? config["imu_delay_ms"].as<double>() : 2.0;
  const auto max_frame_age_ms = config["max_frame_age_ms"]
      ? config["max_frame_age_ms"].as<double>() : 100.0;
  const auto back_camera_config = config["back_camera_config"]
      ? config["back_camera_config"].as<std::string>() : "configs/cam2.yaml";
  if (!std::isfinite(imu_delay_ms) || imu_delay_ms < 0.0 ||
      !std::isfinite(max_frame_age_ms) || max_frame_age_ms <= 0.0)
  {
    throw std::runtime_error("Invalid vision timing configuration");
  }
  const auto imu_delay = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double, std::milli>(imu_delay_ms));
  const auto max_frame_age = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double, std::milli>(max_frame_age_ms));
  rclcpp::init(argc, argv);

  tools::Exiter exiter;
  auto gimbal = std::make_shared<io::GimbalROS>();
  gimbal->start_spin();

  // 初始化相机
  io::SNCamera camera(config_path);
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // 前摄只传递最新帧；后摄的采集与检测由独立工作线程负责。
  CameraStream front_stream(&camera);
  sp_vision::AsyncDetectWorker back_worker("back", back_camera_config, nullptr, 1);
  back_worker.start(config_path);

  // 初始化自瞄与决策模块
  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto tracker = std::make_unique<auto_aim::Tracker>(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);
  omniperception::Decider decider(config_path);

  // 状态机参数
  bool waiting_front_lock = false;
  std::chrono::steady_clock::time_point waiting_front_lock_until = std::chrono::steady_clock::now();
  constexpr int BACK_TO_FRONT_HANDOFF_MS = 800;

  constexpr int BACK_CAM_COOLDOWN_MS = 2000;
  std::chrono::steady_clock::time_point back_cam_cooldown_until = std::chrono::steady_clock::now();

  constexpr int CLEAR_FRONT_ARMOR_FRAMES_AFTER_BACK = 3;
  int clear_front_armor_frames = 0;

  bool last_frame_front_detected_target = false;
  double last_sent_yaw = 0.0;
  double last_sent_pitch = 0.0;
  io::Command last_command{false, false, 0, 0};

  auto clear_front_target_cache = [&]()
  {
    last_frame_front_detected_target = false;
    last_sent_yaw = 0.0;
    last_sent_pitch = 0.0;
    tracker = std::make_unique<auto_aim::Tracker>(config_path, solver);
    clear_front_armor_frames = CLEAR_FRONT_ARMOR_FRAMES_AFTER_BACK;
  };

  double fps = 0.0;
  auto last_fps_time = std::chrono::steady_clock::now();
  int frame_count = 0;

  // 前摄复用图像缓冲区，并用序号避免重复处理同一帧。
  cv::Mat front_img;
  std::uint64_t last_front_seq = 0;
  std::uint64_t last_back_seq = 0;
  auto last_front_frame = std::chrono::steady_clock::now();
  bool front_timeout_reported = false;

  while (!exiter.exit())
  {
    auto now = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point frame_start;
    const bool has_new_front_frame =
        front_stream.get_latest(front_img, frame_start, last_front_seq);
    const bool front_fresh = has_new_front_frame && frame_start <= now &&
        now - frame_start <= max_frame_age;

    bool front_timeout_transition = false;
    if (now - last_front_frame > max_frame_age && !front_timeout_reported)
    {
      // A camera interruption invalidates the old EKF state and the last command.
      tracker = std::make_unique<auto_aim::Tracker>(config_path, solver);
      last_frame_front_detected_target = false;
      front_timeout_reported = true;
      front_timeout_transition = true;
    }

    auto armors = std::list<auto_aim::Armor>{};
    auto targets = std::list<auto_aim::Target>{};
    const auto gimbal_state = gimbal->state();
    Eigen::Vector3d gimbal_pos{
        gimbal_state.yaw, gimbal_state.pitch, 0.0};

    if (front_fresh)
    {
      last_front_frame = frame_start;
      front_timeout_reported = false;

      const Eigen::Quaterniond gimbal_q = gimbal->q(frame_start - imu_delay);
      solver.set_R_gimbal2world(gimbal_q);
      gimbal_pos = tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0);

      armors = yolo.detect(front_img, 0);
      if (clear_front_armor_frames > 0)
      {
        armors.clear();
        --clear_front_armor_frames;
      }

      decider.armor_filter(armors);
      decider.set_priority(armors);
      targets = tracker->track(armors, frame_start);
    }

    io::Command command{false, false, 0, 0};
    now = std::chrono::steady_clock::now();
    const bool front_result_fresh = front_fresh && now - frame_start <= max_frame_age;
    if (front_fresh && !front_result_fresh && !front_timeout_reported)
    {
      // 推理耗时也计入帧龄；结果过期时立刻撤销旧跟踪状态。
      tracker = std::make_unique<auto_aim::Tracker>(config_path, solver);
      last_frame_front_detected_target = false;
      front_timeout_reported = true;
      front_timeout_transition = true;
    }

    const bool front_has_detection = front_result_fresh && !armors.empty();
    const bool front_tracker_lost =
        now - last_front_frame > max_frame_age || tracker->state() == "lost";

    if (!front_result_fresh && !front_tracker_lost)
    {
      back_worker.set_detection_enabled(false);
      std::this_thread::sleep_for(1ms);
      continue;
    }

    bool command_from_front = false;
    bool command_from_back = false;

    // ===== 新增：只保存“前摄当前画面”给出的 xyz =====
    bool need_publish_front_xyz = false;
    Eigen::Vector3d publish_xyz_camera = Eigen::Vector3d::Zero();
    Eigen::Vector3d publish_xyz_gimbal = Eigen::Vector3d::Zero();

    // 注意：必须同时满足
    // 1. 前摄 tracker 没有 lost，也就是前摄锁定
    // 2. 当前这一帧前摄 armors 不为空
    // 这样可以避免 tracker 靠历史状态维持时继续发旧 xyz
    if (!front_tracker_lost && front_has_detection)
    {
      const auto &best_armor = armors.front();

      publish_xyz_gimbal = best_armor.xyz_in_gimbal;
      publish_xyz_camera =
          solver.R_camera2gimbal().transpose() *
          (publish_xyz_gimbal - solver.t_camera2gimbal());

      need_publish_front_xyz = true;
    }

    // =========================
    // 自瞄与决策控制逻辑
    // =========================
    if (front_result_fresh && !front_tracker_lost)
    {
      // 前摄已锁定
      waiting_front_lock = false;

      // 只要前摄在锁定目标，持续刷新后摄冷却时间
      back_cam_cooldown_until = now + std::chrono::milliseconds(BACK_CAM_COOLDOWN_MS);

      const double bullet_speed =
          std::isfinite(gimbal_state.bullet_speed) && gimbal_state.bullet_speed >= 17.0
              ? gimbal_state.bullet_speed : 19.6;
      command = aimer.aim(targets, frame_start, bullet_speed, true);
      command.shoot =
          shooter.shoot(command, aimer, targets, gimbal_pos, tracker->state()) &&
          front_has_detection;
      command_from_front = true;
    }
    else
    {
      // 前摄彻底丢失
      const bool back_allowed =
          (!waiting_front_lock || now >= waiting_front_lock_until) &&
          now > back_cam_cooldown_until;
      back_worker.set_detection_enabled(back_allowed);
      if (back_allowed)
      {
        sp_vision::DetectPacket back_packet;
        if (back_worker.latest(back_packet) && back_packet.seq != last_back_seq)
        {
          last_back_seq = back_packet.seq;
          if (back_packet.ts <= now && now - back_packet.ts <= max_frame_age)
          {
            const auto back_state = gimbal->state();
            const Eigen::Vector3d back_gimbal_pos{back_state.yaw, back_state.pitch, 0.0};
            command = decider.decide_by_armors(back_packet.armors, back_gimbal_pos, "back");

            if (command.control)
            {
              command_from_back = true;
              waiting_front_lock = true;
              waiting_front_lock_until = now + std::chrono::milliseconds(BACK_TO_FRONT_HANDOFF_MS);
              back_cam_cooldown_until = now + std::chrono::milliseconds(BACK_CAM_COOLDOWN_MS);

              armors.clear();
              targets.clear();
              clear_front_target_cache();
              back_worker.set_detection_enabled(false);
            }
          }
        }
      }
      else
      {
        back_worker.set_detection_enabled(false);
      }
    }

    // =========================
    // 发送逻辑 (完全保持你的电控安全逻辑)
    // =========================
    if (command.control)
    {
      gimbal->send(command.control, command.shoot, command.yaw, 0, 0, command.pitch, 0, 0);
      last_command = command;
      // 后摄 command、等待接管、丢目标保持角度，都不会进这里
      if (command_from_front && need_publish_front_xyz)
      {
        gimbal->publish_target_xyz(
            publish_xyz_camera,
            publish_xyz_gimbal,
            gimbal->now());
      }

      if (command_from_front && front_has_detection)
      {
        last_sent_yaw = command.yaw;
        last_sent_pitch = command.pitch;
        last_frame_front_detected_target = true;
      }
      else if (command_from_back)
      {
        last_frame_front_detected_target = false;
        last_sent_yaw = 0.0;
        last_sent_pitch = 0.0;
      }
    }
    else if (front_timeout_transition)
    {
      // 图像中断后明确撤销开火，不能继续沿用上一帧命令。
      gimbal->send(false, false, 0, 0, 0, 0, 0, 0);
    }
    else if (front_result_fresh && last_frame_front_detected_target &&
             !waiting_front_lock && clear_front_armor_frames == 0)
    {
      // 刚丢目标的第一帧，保持上一帧角度
      gimbal->send(true, false, last_sent_yaw, 0, 0, last_sent_pitch, 0, 0);
      last_frame_front_detected_target = false;
    }

    // =========================
    // FPS 监控 (极简版)
    // =========================
    if (front_fresh)
      frame_count++;
    auto loop_end = std::chrono::steady_clock::now();
    double elapsed = tools::delta_time(loop_end, last_fps_time);

    if (elapsed >= 1.0)
    {
      fps = frame_count / elapsed;
      frame_count = 0;
      last_fps_time = loop_end;

      // 【优化】使用 \r 实现原位覆盖刷新，不再刷屏阻塞终端
      fmt::print("\r[Vision] FPS: {:.1f}", fps);
      fflush(stdout);
    }
    if (!front_fresh && !command_from_back)
      std::this_thread::sleep_for(1ms);
  }

  fmt::print("\n");
  rclcpp::shutdown();
  return 0;
}

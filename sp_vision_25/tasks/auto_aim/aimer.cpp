#include "aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
Aimer::Aimer(const std::string & config_path)
: left_yaw_offset_(std::nullopt), right_yaw_offset_(std::nullopt)
{
  auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;        // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;    // degree to rad
  comming_angle_ = yaml["comming_angle"].as<double>() / 57.3;  // degree to rad
  leaving_angle_ = yaml["leaving_angle"].as<double>() / 57.3;  // degree to rad
  high_speed_delay_time_ = yaml["high_speed_delay_time"].as<double>();
  low_speed_delay_time_ = yaml["low_speed_delay_time"].as<double>();
  decision_speed_ = yaml["decision_speed"].as<double>();
  prediction_max_iterations_ = yaml["prediction_max_iterations"]
    ? yaml["prediction_max_iterations"].as<int>() : 10;
  const double fly_time_convergence_ms = yaml["fly_time_convergence_ms"]
    ? yaml["fly_time_convergence_ms"].as<double>() : 1.0;
  fly_time_convergence_s_ = fly_time_convergence_ms / 1000.0;
  if (prediction_max_iterations_ <= 0 || prediction_max_iterations_ > 20 ||
      !std::isfinite(fly_time_convergence_s_) || fly_time_convergence_s_ <= 0.0) {
    throw std::runtime_error("Invalid future hit prediction configuration");
  }
  if (yaml["left_yaw_offset"].IsDefined() && yaml["right_yaw_offset"].IsDefined()) {
    left_yaw_offset_ = yaml["left_yaw_offset"].as<double>() / 57.3;    // degree to rad
    right_yaw_offset_ = yaml["right_yaw_offset"].as<double>() / 57.3;  // degree to rad
    tools::logger()->info("[Aimer] successfully loading shootmode");
  }
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  bool to_now)
{
  // 每次瞄准都先撤销上一帧候选，任何提前返回都不会遗留旧的开火许可。
  shot_candidate_ = {};
  debug_aim_point = {};
  debug_selected_delta_angle = 999.0;
  debug_selected_armor_id = -1;
  if (targets.empty()) return {false, false, 0, 0};
  auto target = targets.front();

  auto ekf = target.ekf();

  double angular_speed = std::abs(target.ekf_x()[7]);
  
  double delay_time =
    angular_speed > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  if (bullet_speed < 14) bullet_speed = 23;

  // 考虑detecor和tracker所消耗的时间，此外假设aimer的用时可忽略不计
  auto future = timestamp;
  if (to_now) {
    double dt;
    dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + delay_time;
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  else {
    auto dt = 0.005 + delay_time;  //detector-aimer耗时0.005+发弹延时0.1
    // tools::logger()->info("dt is {:.4f} second", dt);
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  auto aim_point0 = choose_aim_point(target);
  debug_aim_point = aim_point0;
  if (!aim_point0.valid) {
    // tools::logger()->debug("Invalid aim_point0.");
    return {false, false, 0, 0};
  }

  Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
  auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
  tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
  if (trajectory0.unsolvable) {
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  // 迭代求解子弹到达时的目标位置。飞行时间和物理装甲面必须同时稳定，才允许开火。
  bool converged = false;
  double prev_fly_time = trajectory0.fly_time;
  tools::Trajectory current_traj = trajectory0;
  int previous_armor_id = aim_point0.armor_id;

  for (int iter = 0; iter < prediction_max_iterations_; ++iter) {
    // 预测目标在 future + prev_fly_time 时刻的位置
    auto predict_time = future + std::chrono::microseconds(static_cast<int>(prev_fly_time * 1e6));
    Target iteration_target = target;
    iteration_target.predict(predict_time);

    // 在预计命中时刻重新展开所有物理装甲面，并按原有规则选择真正的命中面。
    auto aim_point = choose_aim_point(iteration_target);
    debug_aim_point = aim_point;
    if (!aim_point.valid) {
      return {false, false, 0, 0};
    }

    // 计算新弹道
    Eigen::Vector3d xyz = aim_point.xyza.head(3);
    double d = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
    current_traj = tools::Trajectory(bullet_speed, d, xyz.z());

    // 检查弹道是否可解
    if (current_traj.unsolvable) {
      tools::logger()->debug(
        "[Aimer] Unsolvable trajectory in iter {}: speed={:.2f}, d={:.2f}, z={:.2f}", iter + 1,
        bullet_speed, d, xyz.z());
      debug_aim_point.valid = false;
      return {false, false, 0, 0};
    }

    // 只看飞行时间可能在切板瞬间误判收敛，因此同时要求未来命中面编号稳定。
    const bool fly_time_stable =
      std::abs(current_traj.fly_time - prev_fly_time) < fly_time_convergence_s_;
    const bool face_stable = aim_point.armor_id == previous_armor_id;
    if (fly_time_stable && face_stable) {
      converged = true;
      break;
    }
    prev_fly_time = current_traj.fly_time;
    previous_armor_id = aim_point.armor_id;
  }

  // 计算最终角度
  Eigen::Vector3d final_xyz = debug_aim_point.xyza.head(3);
  double yaw = std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;
  double pitch = -(current_traj.pitch + pitch_offset_);  //世界坐标系下pitch向上为负

  shot_candidate_.trajectory_converged = converged;
  shot_candidate_.armor_id = debug_aim_point.armor_id;
  shot_candidate_.hit_xyza = debug_aim_point.xyza;
  shot_candidate_.view_angle = debug_aim_point.view_angle;
  shot_candidate_.fly_time = current_traj.fly_time;
  shot_candidate_.valid = converged && debug_aim_point.valid &&
                          debug_aim_point.armor_id >= 0 && final_xyz.allFinite() &&
                          std::isfinite(yaw) && std::isfinite(pitch) &&
                          std::isfinite(current_traj.fly_time);
  if (!shot_candidate_.valid) {
    tools::logger()->debug(
      "[Aimer][FutureHit] no fire candidate: converged={} face={} fly_time={:.4f}s",
      converged, debug_aim_point.armor_id, current_traj.fly_time);
  }
  return {true, false, yaw, pitch};
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  io::ShootMode shoot_mode, bool to_now)
{
  double yaw_offset;
  if (shoot_mode == io::left_shoot && left_yaw_offset_.has_value()) {
    yaw_offset = left_yaw_offset_.value();
  } else if (shoot_mode == io::right_shoot && right_yaw_offset_.has_value()) {
    yaw_offset = right_yaw_offset_.value();
  } else {
    yaw_offset = yaw_offset_;
  }

  auto command = aim(targets, timestamp, bullet_speed, to_now);
  command.yaw = command.yaw - yaw_offset_ + yaw_offset;

  return command;
}

AimPoint Aimer::choose_aim_point(const Target & target)
{
  Eigen::VectorXd ekf_x = target.ekf_x();
  std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  auto armor_num = armor_xyza_list.size();
  if (armor_num == 0) return {};

  // 整车旋转中心的球坐标yaw
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
  std::vector<double> delta_angle_list;
  for (int i = 0; i < armor_num; i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.emplace_back(delta_angle);
  }

  debug_selected_delta_angle = 999.0;
  debug_selected_armor_id = -1;

  auto select_armor = [&](int armor_id) {
    debug_selected_delta_angle = delta_angle_list[armor_id];
    debug_selected_armor_id = armor_id;
    return AimPoint{
      true, armor_xyza_list[armor_id], armor_id, delta_angle_list[armor_id]};
  };

  // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
  if (!target.jumped) {
    return select_armor(0);
  }

  // 不考虑小陀螺
  constexpr double SPIN_THRESHOLD = 7;
  if (std::abs(target.ekf_x()[7]) < SPIN_THRESHOLD && target.name != ArmorName::outpost) {
    // 选择在可射击范围内的装甲板
    std::vector<int> id_list;
    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[i]) > 50 / 57.3) continue;
      id_list.push_back(i);
    }
    // 绝无可能
    if (id_list.empty()) {
      tools::logger()->warn("Empty id list!");
      return {};
    }

    // 锁定模式：防止在两个都呈45度的装甲板之间来回切换
    if (id_list.size() > 1) {
      int id0 = id_list[0], id1 = id_list[1];

      // 初次进入锁定：选绝对角度更小的那块
      if (lock_id_ != id0 && lock_id_ != id1) {
        lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;
      } else {
        // 已经锁定时，只有另一块明显更优才允许切换
        int other_id = (lock_id_ == id0) ? id1 : id0;
        double locked_abs = std::abs(delta_angle_list[lock_id_]);
        double other_abs  = std::abs(delta_angle_list[other_id]);

        constexpr double SWITCH_HYSTERESIS = 8.0 / 57.3;  // 8度迟滞
        if (other_abs + SWITCH_HYSTERESIS < locked_abs) {
          lock_id_ = other_id;
        }
      }

      return select_armor(lock_id_);
    }

    // 只有一个装甲板在可射击范围内时，退出锁定模式
    lock_id_ = -1;
    return select_armor(id_list[0]);
  }

  double coming_angle, leaving_angle;
  if (target.name == ArmorName::outpost) {
    coming_angle = 70 / 57.3;
    leaving_angle = 30 / 57.3;
  } else {
    coming_angle = comming_angle_;
    leaving_angle = leaving_angle_;
  }

  // 在小陀螺时，一侧的装甲板不断出现，另一侧的装甲板不断消失，显然前者被打中的概率更高
  for (int i = 0; i < armor_num; i++) {
    if (std::abs(delta_angle_list[i]) > coming_angle) continue;

    if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle) {
      return select_armor(i);
    }

    if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle) {
      return select_armor(i);
    }
  }

  return {};
}

}  // namespace auto_aim

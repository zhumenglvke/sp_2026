#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <optional>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "target.hpp"

namespace auto_aim
{

struct AimPoint
{
  bool valid{false};
  Eigen::Vector4d xyza{Eigen::Vector4d::Zero()};
  int armor_id{-1};
  double view_angle{0.0};
};

// 描述子弹预计到达时真正使用的物理装甲面。Shooter 只对有效候选放行。
struct ShotCandidate
{
  bool valid{false};
  bool trajectory_converged{false};
  int armor_id{-1};
  Eigen::Vector4d hit_xyza{Eigen::Vector4d::Zero()};
  double view_angle{0.0};
  double fly_time{0.0};
};

class Aimer
{
public:
  AimPoint debug_aim_point;
  double debug_selected_delta_angle = 999.0;
  int debug_selected_armor_id = -1;
  explicit Aimer(const std::string & config_path);
  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now = true);

  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    io::ShootMode shoot_mode, bool to_now = true);

  const ShotCandidate & shot_candidate() const noexcept { return shot_candidate_; }

private:
  double yaw_offset_;
  std::optional<double> left_yaw_offset_, right_yaw_offset_;
  double pitch_offset_;
  double comming_angle_;
  double leaving_angle_;
  int lock_id_ = -1;
  double high_speed_delay_time_;
  double low_speed_delay_time_;
  double decision_speed_;
  int prediction_max_iterations_{10};
  double fly_time_convergence_s_{0.001};
  ShotCandidate shot_candidate_{};

  AimPoint choose_aim_point(const Target & target);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP

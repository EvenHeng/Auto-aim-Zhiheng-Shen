#pragma once

#include <chrono>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include "rm_aim_sim_bridge/msg/aim_command.hpp"
#include "rm_aim_sim_bridge/msg/armor_corners_array.hpp"
#include "rm_aim_sim_bridge/msg/gimbal_state.hpp"
#include "rm_aim_sim_bridge/msg/telemetry.hpp"

namespace student_auto_aim
{

// 极简三维向量，避免引入额外数学库依赖
struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

Vec3 operator+(const Vec3 & a, const Vec3 & b);
Vec3 operator-(const Vec3 & a, const Vec3 & b);
Vec3 operator*(const Vec3 & a, double s);
double dot(const Vec3 & a, const Vec3 & b);
Vec3 cross(const Vec3 & a, const Vec3 & b);
double norm(const Vec3 & a);
Vec3 normalize(const Vec3 & a);
Vec3 rotateZ(const Vec3 & v, double angle);
double wrapPi(double a);

class AimNode : public rclcpp::Node
{
public:
  AimNode();

private:
  using ArmorCornersArray = rm_aim_sim_bridge::msg::ArmorCornersArray;
  using GimbalState = rm_aim_sim_bridge::msg::GimbalState;
  using Telemetry = rm_aim_sim_bridge::msg::Telemetry;
  using AimCommand = rm_aim_sim_bridge::msg::AimCommand;

  // 一帧里的一块装甲：中心点、朝外法向、正对程度
  struct Plate
  {
    Vec3 center;
    Vec3 normal;
    double facing{0.0};
  };

  // 运动样本：c 存的是机器人中心 q = 板心 - r·法向，换板时 q 连续，速度估计不被打断
  struct Sample
  {
    double t{0.0};
    Vec3 c;
    Vec3 n;
  };

  struct PointSample
  {
    double t{0.0};
    Vec3 q;
  };

  // ---------- 回调 ----------
  void onArmors(const ArmorCornersArray::ConstSharedPtr msg);
  void onGimbal(const GimbalState::ConstSharedPtr msg);
  void onTelemetry(const Telemetry::ConstSharedPtr msg);
  void controlLoop();

  // ---------- 算法 ----------
  static Vec3 muzzleAt(double yaw, double pitch, double barrel_length);
  std::vector<Plate> extractPlates(const ArmorCornersArray & msg, const Vec3 & muzzle) const;
  int selectPlate(const std::vector<Plate> & plates, const Vec3 & muzzle, double dt,
                  double lead) const;
  void addSample(double stamp, const Vec3 & c, const Vec3 & n);
  void updateOmega(double stamp, const Vec3 & n);
  void fitMotion();
  Vec3 predictCenter(double dt) const;
  bool solveAim(double age, const Telemetry & tel, double yaw_hint, double pitch_hint,
                double & yaw_out, double & pitch_out) const;
  // 解算 → 到位判定 → 发布指令；allow_fire=false 时只跟瞄不开火
  void aimAndFire(const std_msgs::msg::Header & header, const std::string & session_id,
                  const GimbalState & gimbal, const Telemetry & tel, double now, double age,
                  const Vec3 & center_hint, int sel, bool allow_fire);
  // 当前帧没有可用装甲（空列表或只剩侧对板），但预测仍然可信时，允许继续按预测位置打
  bool canHold(double stamp, double grace) const;
  void resetTrack();
  void publishCommand(const std_msgs::msg::Header & header, const std::string & session_id,
                      double yaw, double pitch, bool enable, bool fire);
  double simNow();

  // ---------- ROS 接口 ----------
  rclcpp::Subscription<ArmorCornersArray>::SharedPtr armors_sub_;
  rclcpp::Subscription<GimbalState>::SharedPtr gimbal_sub_;
  rclcpp::Subscription<Telemetry>::SharedPtr telemetry_sub_;
  rclcpp::Publisher<AimCommand>::SharedPtr command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---------- 共享数据 ----------
  std::mutex mtx_;
  ArmorCornersArray::ConstSharedPtr armors_;
  GimbalState::ConstSharedPtr gimbal_;
  Telemetry telemetry_{};
  std::chrono::steady_clock::time_point last_armors_wall_{};

  // ---------- 会话与跟踪状态 ----------
  std::string session_id_;
  double last_stamp_{-1.0};

  std::deque<Sample> history_;
  std::deque<double> psi_time_;
  std::deque<double> psi_value_;
  bool psi_valid_{false};
  double psi_prev_{0.0};
  double omega_{0.0};

  Vec3 p_hat_{};
  Vec3 v_hat_{};
  Vec3 a_hat_{};
  double r_track_{0.0};
  double p_hat_time_{0.0};
  bool fit_ok_{false};
  bool motion_ready_{false};  // 当前拟合用到了足够的窗口内样本（速度估计可信）
  double fit_rms_{0.0};       // 二次拟合的残差 RMS（m），越大说明窗口内跨了运动段

  Vec3 last_plate_center_{};
  Vec3 last_plate_normal_{};
  bool last_plate_valid_{false};

  double last_yaw_cmd_{0.0};
  double last_pitch_cmd_{0.0};
  bool last_cmd_valid_{false};

  // ---------- 参数 ----------
  double control_rate_{50.0};
  double aim_tolerance_{0.02};
  double extra_latency_{0.02};
  double fit_window_{0.35};
  double trans_window_{0.20};
  int max_history_{45};
  double omega_deadband_{0.3};
  double facing_min_{0.5};
  double facing_hold_{0.4};
  double plate_jump_gate_{0.20};
  double hold_grace_{0.30};        // 只剩侧对板时按预测继续射击的宽限（s）
  double hold_grace_empty_{0.12};  // 装甲列表整体为空时的宽限（s，覆盖灭灯，击毁后不再空枪）
  double pitch_min_{-0.5236};
  double pitch_max_{0.3491};
};

}  // namespace student_auto_aim

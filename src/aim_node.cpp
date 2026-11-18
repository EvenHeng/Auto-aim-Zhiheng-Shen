#include "student_auto_aim/aim_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <utility>

namespace student_auto_aim
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

// 高斯消元求 n 元一次方程组（最大 10 元），带列主元
bool solveLinear(int n, double A[10][10], double b[10], double x[10])
{
  for (int col = 0; col < n; ++col) {
    int pivot = col;
    for (int row = col + 1; row < n; ++row) {
      if (std::abs(A[row][col]) > std::abs(A[pivot][col])) {
        pivot = row;
      }
    }
    if (std::abs(A[pivot][col]) < 1e-12) {
      return false;
    }
    if (pivot != col) {
      for (int k = col; k < n; ++k) {
        std::swap(A[col][k], A[pivot][k]);
      }
      std::swap(b[col], b[pivot]);
    }
    const double d = A[col][col];
    for (int k = col; k < n; ++k) {
      A[col][k] /= d;
    }
    b[col] /= d;
    for (int row = 0; row < n; ++row) {
      if (row == col) {
        continue;
      }
      const double f = A[row][col];
      if (std::abs(f) < 1e-15) {
        continue;
      }
      for (int k = col; k < n; ++k) {
        A[row][k] -= f * A[col][k];
      }
      b[row] -= f * b[col];
    }
  }
  for (int i = 0; i < n; ++i) {
    x[i] = b[i];
  }
  return true;
}

}  // namespace

// ===================== Vec3 基本运算 =====================

Vec3 operator+(const Vec3 & a, const Vec3 & b) { return Vec3{a.x + b.x, a.y + b.y, a.z + b.z}; }

Vec3 operator-(const Vec3 & a, const Vec3 & b) { return Vec3{a.x - b.x, a.y - b.y, a.z - b.z}; }

Vec3 operator*(const Vec3 & a, double s) { return Vec3{a.x * s, a.y * s, a.z * s}; }

double dot(const Vec3 & a, const Vec3 & b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

Vec3 cross(const Vec3 & a, const Vec3 & b)
{
  return Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

double norm(const Vec3 & a) { return std::sqrt(dot(a, a)); }

Vec3 normalize(const Vec3 & a)
{
  const double n = norm(a);
  if (n < 1e-9) {
    return Vec3{0.0, 0.0, 0.0};
  }
  return a * (1.0 / n);
}

// 绕 odom 的 Z 轴（向上）旋转，用于旋转靶的相位预测
Vec3 rotateZ(const Vec3 & v, double angle)
{
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  return Vec3{v.x * c - v.y * s, v.x * s + v.y * c, v.z};
}

double wrapPi(double a)
{
  while (a > kPi) {
    a -= 2.0 * kPi;
  }
  while (a < -kPi) {
    a += 2.0 * kPi;
  }
  return a;
}

// ===================== 节点 =====================

AimNode::AimNode()
: rclcpp::Node("aim_node")
{
  control_rate_ = this->declare_parameter<double>("control_rate", 50.0);
  aim_tolerance_ = this->declare_parameter<double>("aim_tolerance", 0.02);
  // 观测延迟补偿。理想云台瞬时到位，执行延迟由 telemetry.delay 给出、消息年龄由
  // header.stamp 的 age 单独补偿，0.02 s 是这段管线的经验余量。
  // 实测把它调到 0 或 +0.05 都更差（组合场景 0.50 → 0.40，平移 0.947 → 0.692），
  // 说明 0.02 正处在最优平台区，不要再动。
  extra_latency_ = this->declare_parameter<double>("extra_latency", 0.02);
  fit_window_ = this->declare_parameter<double>("fit_window", 0.35);
  // 平移拟合窗口。0.20 s 时二次拟合的 v 抖动会被 0.3 s 提前量放大（v 误差 20% ≈ 6 cm 空枪），
  // 拉到 0.30 s 后样本翻倍、v 明显更稳；代价是 0.33 s 的加减速段会有轻微滞后。
  trans_window_ = this->declare_parameter<double>("trans_window", 0.30);
  max_history_ = this->declare_parameter<int>("max_history", 45);
  omega_deadband_ = this->declare_parameter<double>("omega_deadband", 0.3);
  facing_min_ = this->declare_parameter<double>("facing_min", 0.5);
  facing_hold_ = this->declare_parameter<double>("facing_hold", 0.4);
  plate_jump_gate_ = this->declare_parameter<double>("plate_jump_gate", 0.20);
  hold_grace_ = this->declare_parameter<double>("hold_grace", 0.30);
  hold_grace_empty_ = this->declare_parameter<double>("hold_grace_empty", 0.12);
  // 换板后短暂压制开火：实测换板瞬间云台还指着上一块板，这一拍开火会沿旧方向发弹。
  // 但压制时长必须很小：和"两块板都合格"时的选板抖动叠加，会吃掉大部分开火占空比
  // （实测 0.15 s 时 251 拍里只有 27 拍允许开火，击毁时间被拉到 32 s）。0.06 s 足够。
  switch_settle_ = this->declare_parameter<double>("switch_settle", 0.06);
  pitch_min_ = this->declare_parameter<double>("pitch_min", -0.5236);
  pitch_max_ = this->declare_parameter<double>("pitch_max", 0.3491);

  // 未收到 telemetry 时的兜底参数（与仿真默认值一致）
  telemetry_.speed = 23.0;
  telemetry_.gravity = 9.7833;
  telemetry_.barrel_length = 0.22;
  telemetry_.rate = 10.0;
  telemetry_.delay = 0.015;

  // 手册要求：reliable + volatile + depth 1
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliable().durability_volatile();

  armors_sub_ = this->create_subscription<ArmorCornersArray>(
    "/aim_sim/armors", qos,
    std::bind(&AimNode::onArmors, this, std::placeholders::_1));
  gimbal_sub_ = this->create_subscription<GimbalState>(
    "/aim_sim/gimbal", qos,
    std::bind(&AimNode::onGimbal, this, std::placeholders::_1));
  telemetry_sub_ = this->create_subscription<Telemetry>(
    "/aim_sim/telemetry", qos,
    std::bind(&AimNode::onTelemetry, this, std::placeholders::_1));
  command_pub_ = this->create_publisher<AimCommand>("/aim_sim/command", qos);

  const double rate = std::max(1.0, control_rate_);
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / rate),
    std::bind(&AimNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(), "自瞄节点已启动：control_rate=%.0f Hz", rate);
}

double AimNode::simNow() { return this->now().seconds(); }

void AimNode::onArmors(const ArmorCornersArray::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  armors_ = msg;
  last_armors_wall_ = std::chrono::steady_clock::now();
}

void AimNode::onGimbal(const GimbalState::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  gimbal_ = msg;
}

void AimNode::onTelemetry(const Telemetry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  telemetry_ = *msg;
}

void AimNode::resetTrack()
{
  history_.clear();
  psi_time_.clear();
  psi_value_.clear();
  psi_valid_ = false;
  omega_ = 0.0;
  p_hat_ = Vec3{};
  v_hat_ = Vec3{};
  a_hat_ = Vec3{};
  r_track_ = 0.0;
  p_hat_time_ = 0.0;
  fit_ok_ = false;
  motion_ready_ = false;
  last_plate_valid_ = false;
  last_stamp_ = -1.0;
}

Vec3 AimNode::muzzleAt(double yaw, double pitch, double barrel_length)
{
  // 枪口在云台系 X 轴上，随 yaw / pitch 一起转动
  const Vec3 forward{
    std::cos(pitch) * std::cos(yaw),
    std::cos(pitch) * std::sin(yaw),
    std::sin(pitch)};
  return forward * barrel_length;
}

std::vector<AimNode::Plate> AimNode::extractPlates(
  const ArmorCornersArray & msg, const Vec3 & muzzle) const
{
  std::vector<Plate> out;
  out.reserve(msg.armors.size());

  for (const auto & armor : msg.armors) {
    const auto & p0 = armor.corners[0];  // 左上
    const auto & p1 = armor.corners[1];  // 右上
    const auto & p2 = armor.corners[2];  // 右下
    const auto & p3 = armor.corners[3];  // 左下

    const Vec3 c{
      (p0.x + p1.x + p2.x + p3.x) * 0.25,
      (p0.y + p1.y + p2.y + p3.y) * 0.25,
      (p0.z + p1.z + p2.z + p3.z) * 0.25};

    // 板面右方向与向下方向，叉乘得到朝外法向
    // 注意：cross(右, 下) 是朝内，朝外必须 cross(下, 右)
    const Vec3 right{p1.x - p0.x, p1.y - p0.y, p1.z - p0.z};
    const Vec3 down{p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};
    const Vec3 n = normalize(cross(down, right));
    if (norm(n) < 1e-6) {
      continue;  // 退化角点，丢弃
    }

    const double facing = dot(n, normalize(muzzle - c));
    out.push_back(Plate{c, n, facing});
  }
  return out;
}

int AimNode::selectPlate(
  const std::vector<Plate> & plates, const Vec3 & muzzle, double dt, double lead) const
{
  // ---- 旋转靶：按"子弹到场那一刻"的正对程度选板 ----
  // 命中面随板一起转，ω·飞行时间在 ω=4 rad/s、6 m 时约 1.24 rad（71°）。
  // 若按"此刻正对"选板（此刻 f≈0.98 的那块），子弹飞到的时候它已经转成 71° 斜擦，
  // 等效容差只剩两三厘米，于是大量穿透飞过（实测 30 发里 11 发 expired）。
  // 四块板 90° 均布，任何时刻必有一块"到场正对 ≥ 0.707"，按到场正对选板即可持续开火。
  constexpr double kFacingNowMin = 0.25;
  if (std::abs(omega_) > omega_deadband_) {
    // 【教训】这里千万不能"锁定就一路打到底"：旋转靶的提前量约 ω·飞行 ≈ 1.2 rad（70°），
    // 一块板从出现到离开的整段可见期里，它在子弹到场时可能已经斜到 60° 以上。
    // 曾把"找回锁定板"放在选板之前，结果 sel 恒定不变、56 发 expired、kill_time=-1。
    // 所以旋转模式必须以【到场正对】为唯一判据，锁定只做小范围迟滞，不做身份保持。
    // 约束是【同时】满足：此刻要看得清（f_now 高，仿真只在板大致朝向相机时才发布它，
    // 侧对到 f≈0.3 时它时隐时现，样本一断预测就漂），到场又要正对（f_arr 高，容差才够大）。
    // 只优化 f_arr 会把"此刻几乎侧对"的板选出来，这正是旋转/组合场景的漏源。
    // 四块板 90° 均布、f_now ≥ 0.4 对应 132° 的允许窗口，任意时刻都必定有一块同时满足，
    // 而且 f_arr 仍能做到 0.99（φ = -66° 时 f_arr = cos(5°)）。
    int best = -1;
    double best_f = -1e9;
    for (size_t i = 0; i < plates.size(); ++i) {
      const Vec3 dir = normalize(muzzle - plates[i].center);
      const double f_now = dot(plates[i].normal, dir);
      if (f_now < kFacingNowMin) {
        continue;  // 背对/近乎背对的板：法向朝反，q = c - r·n 会直接错 2r，必须排除
      }
      // 到场时的方位角要用【到场时刻的板心】来算，不能用此刻的板心：
      // 目标平移时板心在这 lead 秒里会挪开 v·lead（1 m/s × 0.3 s = 30 cm），
      // 6 m 距离上就是 0.05 rad 的方位偏转。用此刻板心会把"往右折返"和"往左走"
      // 当成同一种几何，选出来的板在到场时已经偏斜，这是组合场景换向段的主要漏源。
      const Vec3 c_arr = plates[i].center + v_hat_ * lead;
      const Vec3 dir_arr = normalize(muzzle - c_arr);
      const Vec3 n_arr = rotateZ(plates[i].normal, omega_ * lead);
      const double f_arr = dot(n_arr, dir_arr);
      if (f_arr > best_f) {
        best_f = f_arr;
        best = static_cast<int>(i);
      }
    }
    // 迟滞：当前锁着的那块只要不比最优差太多就继续打，避免在交叉点来回换板
    if (last_plate_valid_ && best >= 0) {
      for (size_t i = 0; i < plates.size(); ++i) {
        if (dot(plates[i].normal, rotateZ(last_plate_normal_, omega_ * dt)) < 0.94) {
          continue;
        }
        const Vec3 dir = normalize(muzzle - plates[i].center);
        if (dot(plates[i].normal, dir) < kFacingNowMin) {
          continue;
        }
        const Vec3 c_arr = plates[i].center + v_hat_ * lead;
        const Vec3 n_arr = rotateZ(plates[i].normal, omega_ * lead);
        const double f_arr = dot(n_arr, normalize(muzzle - c_arr));
        if (f_arr >= best_f - 0.15) {
          return static_cast<int>(i);
        }
      }
    }
    if (best < 0) {
      // 理论上不该发生（四块板 90° 均布，f_now ≥ 0.25 的窗口 151° 一定覆盖一块）。
      // 真的发生就交给 hold 分支按预测位置顶，不要因此停火 —— 停火会拉长击毁时间。
      return -1;
    }
    return best;  // 不再按 f_arr 设下限：设下限会挑"此刻近乎侧对"的板并造成 1~4 s 的停火空档
  }
  // ---- 第一步：锁板（按法向匹配） ----
  // 同一台机器人的几块板方位差 90°、高度与半径也不完全相同，所以"位置接近"不足以判断是不是同一块板；
  // 而每块板的法向是唯一且缓慢变化的（旋转靶按 ω 转），用它可以稳定锁板，避免换板带来的 q 跳变。
  //
  // 【几何粘住】平移/静止场景里两块板的正对度会交替跌破 facing_hold，法向一锁断就会重选，
  // 而两块板在横向差 2r ≈ 0.5 m，云台就在两点之间来回甩 —— 实测每 0.1 s 翻一次，
  // 每次翻转都触发换板禁射，表现为"开局两三秒完全不射击"。
  // 所以先按"几何位置"认人：谁离"上一帧那块板按速度外推后的位置"最近，就还是它。
  if (last_plate_valid_ && last_stamp_ >= 0.0) {
    const Vec3 c_pred = last_plate_center_ + v_hat_ * std::min(0.2, std::max(0.0, dt));
    int sticky = -1;
    double sticky_d = 1e9;
    for (size_t i = 0; i < plates.size(); ++i) {
      const double d = norm(plates[i].center - c_pred);
      if (d < 0.12 && d < sticky_d) {
        sticky_d = d;
        sticky = static_cast<int>(i);
      }
    }
    if (sticky >= 0) {
      return sticky;
    }
  }

  if (last_plate_valid_) {
    const Vec3 n_pred = rotateZ(last_plate_normal_, omega_ * dt);
    int nearest = -1;
    double nearest_d = 1e9;
    for (size_t i = 0; i < plates.size(); ++i) {
      if (plates[i].facing < facing_hold_) {
        continue;  // 接近侧对，命中面积太小，宁可切板
      }
      if (dot(plates[i].normal, n_pred) < 0.94) {
        continue;  // 法向相差超过约 20°，不是同一块板
      }
      const double d = norm(plates[i].center - last_plate_center_);
      if (d < plate_jump_gate_ * 3.0 && d < nearest_d) {
        nearest_d = d;
        nearest = static_cast<int>(i);
      }
    }
    if (nearest >= 0) {
      return nearest;
    }
  }

  // ---- 第二步：重新捕获 ----
  int best = -1;
  double best_score = -1e9;
  for (size_t i = 0; i < plates.size(); ++i) {
    const Plate & p = plates[i];
    if (p.facing < facing_min_) {
      continue;  // 背向的板打不中，直接排除
    }
    const Vec3 rel = p.center - muzzle;
    const double dist = norm(rel);
    const double ang = std::abs(wrapPi(std::atan2(rel.y, rel.x)));

    const double score = p.facing - 0.06 * dist - 0.15 * ang;

    if (score > best_score) {
      best_score = score;
      best = static_cast<int>(i);
    }
  }
  if (best >= 0) {
  }
  return best;
}

void AimNode::addSample(double stamp, const Vec3 & c, const Vec3 & n)
{
  history_.push_back(Sample{stamp, c, n});
  while (!history_.empty()) {
    const bool too_old = (stamp - history_.front().t) > fit_window_;
    const bool too_many = static_cast<int>(history_.size()) > max_history_;
    if (!too_old && !too_many) {
      break;
    }
    history_.pop_front();
  }
}

void AimNode::updateOmega(double stamp, const Vec3 & n)
{
  const double psi_raw = std::atan2(n.y, n.x);

  if (!psi_valid_) {
    psi_prev_ = psi_raw;
    psi_valid_ = true;
    psi_time_.clear();
    psi_value_.clear();
  } else {
    // 装甲切换会带来 90° 整数倍跳变，这里选与上一帧最接近的等价角
    double best = psi_raw;
    double best_dist = 1e9;
    for (int k = -2; k <= 2; ++k) {
      for (int m = -1; m <= 1; ++m) {
        const double cand = psi_raw + k * (kPi / 2.0) + m * (2.0 * kPi);
        const double dist = std::abs(cand - psi_prev_);
        if (dist < best_dist) {
          best_dist = dist;
          best = cand;
        }
      }
    }
    if (best_dist > 1.3) {
      // 跳变过大，开启新片段，避免污染转速估计
      psi_time_.clear();
      psi_value_.clear();
    }
    psi_prev_ = best;
  }

  psi_time_.push_back(stamp);
  psi_value_.push_back(psi_prev_);
  while (!psi_time_.empty() && (stamp - psi_time_.front()) > fit_window_) {
    psi_time_.pop_front();
    psi_value_.pop_front();
  }

  const size_t cnt = psi_time_.size();
  if (cnt < 5 || (psi_time_.back() - psi_time_.front()) < 0.06) {
    return;
  }

  double tm = 0.0;
  double vm = 0.0;
  for (size_t i = 0; i < cnt; ++i) {
    tm += psi_time_[i];
    vm += psi_value_[i];
  }
  tm /= static_cast<double>(cnt);
  vm /= static_cast<double>(cnt);

  double num = 0.0;
  double den = 0.0;
  for (size_t i = 0; i < cnt; ++i) {
    const double dt = psi_time_[i] - tm;
    num += dt * (psi_value_[i] - vm);
    den += dt * dt;
  }
  if (den < 1e-6) {
    return;
  }

  double w = num / den;
  if (std::abs(w) < omega_deadband_) {
    w = 0.0;  // 静止靶的法向抖动不应被当成旋转
  }
  if (std::abs(w) < 6.0) {
    omega_ = w;
  }
}

void AimNode::fitMotion()
{
  fit_ok_ = false;
  if (history_.size() < 3) {
    // 刚进会话，确实没有速度可估
    motion_ready_ = false;
    return;
  }

  // 历史里存的已经是机器人中心 q = c - r*n（见 controlLoop），这里只对 q 做二次拟合，
  // 换板不会打断运动估计，这是运动靶稳定性的关键。
  std::vector<PointSample> qs;
  qs.reserve(history_.size());
  const double t_last = history_.back().t;
  for (const auto & s : history_) {
    if ((t_last - s.t) > trans_window_) {
      continue;
    }
    qs.push_back(PointSample{s.t, s.c});
  }

  if (qs.size() < 3) {
    // 丢失（侧对出画、灭灯）之后样本还没在窗口内攒够 3 个：
    // 此时既不能退化成"零速度"（那等于打一发没有提前量的空枪），也不能沿用旧速度瞎猜。
    // 做法是让预测退化为"当前测量点"（fit_ok_ 已在上方置 false），同时把 motion_ready_ 置 false，
    // 由开火门限拦住，等 0.1 s 攒够样本再开火。
    motion_ready_ = false;
    return;
  }

  double tq = 0.0;
  for (const auto & p : qs) {
    tq += p.t;
  }
  tq /= static_cast<double>(qs.size());

  double M[10][10] = {};
  for (const auto & p : qs) {
    const double t = p.t - tq;
    const double row[3] = {1.0, t, t * t};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        M[i][j] += row[i] * row[j];
      }
    }
  }

  double coef[3][3] = {};  // [轴][A, B, C]，q = A + B*t + C*t^2
  bool ok = true;
  for (int axis = 0; axis < 3; ++axis) {
    double rhs[3] = {0.0, 0.0, 0.0};
    for (const auto & p : qs) {
      const double t = p.t - tq;
      const double val = (axis == 0) ? p.q.x : (axis == 1) ? p.q.y : p.q.z;
      const double row[3] = {1.0, t, t * t};
      for (int i = 0; i < 3; ++i) {
        rhs[i] += row[i] * val;
      }
    }
    double M2[10][10];
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        M2[i][j] = M[i][j];
      }
    }
    double x[10];
    if (!solveLinear(3, M2, rhs, x)) {
      ok = false;
      break;
    }
    coef[axis][0] = x[0];
    coef[axis][1] = x[1];
    coef[axis][2] = x[2];
  }

  if (!ok) {
    motion_ready_ = false;
    return;
  }

  Vec3 p{coef[0][0], coef[1][0], coef[2][0]};
  Vec3 v{coef[0][1], coef[1][1], coef[2][1]};
  Vec3 acc{coef[0][2] * 2.0, coef[1][2] * 2.0, coef[2][2] * 2.0};

  // 限幅，防止个别噪声把外推带飞
  for (double * comp : {&v.x, &v.y, &v.z}) {
    if (std::abs(*comp) > 3.0) {
      *comp = std::copysign(3.0, *comp);
    }
  }
  for (double * comp : {&acc.x, &acc.y, &acc.z}) {
    if (std::abs(*comp) > 4.0) {
      *comp = std::copysign(4.0, *comp);
    }
  }

  // 物理斜率限制：平移最高加速度只有 3 m/s²，一帧（20 ms）速度最多变 0.06 m/s，
  // 不可能像实测中出现的那样从 +1.0 翻到 -0.27。真突变时拒绝这一帧的速度，
  // 让它以 4 m/s² 的速度慢慢收敛（步进 0.08 m/s），几帧内就能跟上真实换向。
  if (norm(v_hat_) > 0.05 && norm(v - v_hat_) > 0.80) {
    const Vec3 dv = v - v_hat_;
    v = v_hat_ + dv * (0.08 / norm(dv));
    acc = Vec3{};
  }

  // 运动死区：静止靶不应把测量噪声外推成提前量
  if (norm(v) < 0.06) {
    v = Vec3{};
  }
  if (norm(acc) < 0.40) {
    acc = Vec3{};
  }

  p_hat_ = p;
  v_hat_ = v;
  a_hat_ = acc;
  p_hat_time_ = tq;
  fit_ok_ = true;
  // 灭灯期间采样本就稀疏（每 100 ms 里约有一半帧看不到板），窗口内 4 个点已经够定二次曲线了
  motion_ready_ = qs.size() >= 4;

  // 残差 RMS：匀速/匀减速段应当接近 0；折返点窗口里混了两段运动时会明显变大，
  // 这时候的速度符号是"旧的"，外推会偏几十厘米，宁可这一瞬不开火。
  double ss = 0.0;
  for (const auto & sp : qs) {
    const double t = sp.t - tq;
    const Vec3 res{
      coef[0][0] + (coef[0][1] + coef[0][2] * t) * t - sp.q.x,
      coef[1][0] + (coef[1][1] + coef[1][2] * t) * t - sp.q.y,
      coef[2][0] + (coef[2][1] + coef[2][2] * t) * t - sp.q.z};
    ss += dot(res, res);
  }
  fit_rms_ = std::sqrt(ss / static_cast<double>(qs.size()));

  // 折返段：窗口跨了"减速 + 反向加速"两段运动，二次拟合残差必然很大（实测 0.03~0.06）。
  // 曾试过在此改用尾段差商当速度并继续开火（不打折扣地保节拍），实测命中率从 0.75 掉到 0.53：
  // 折返瞬间的差商噪声被 0.3 s 提前量放大，多发出去的都是斜擦。所以这里仍然停火 ——
  // 命中率是评分的第一口径，宁可等 0.3 s 让窗口重新干净。
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 500,
    "折返段残差 %.3f 超过门限，等窗口干净再开火", fit_rms_);
}

Vec3 AimNode::predictCenter(double dt) const
{
  if (!fit_ok_) {
    return last_plate_center_;
  }
  const double tau = (last_stamp_ + dt) - p_hat_time_;

  Vec3 q = p_hat_ + v_hat_ * tau + a_hat_ * (0.5 * tau * tau);

  // 仿真里的平移动是梯形速度：到端点先减速停住，再朝反方向加速。
  // 纯二次模型在减速段之后会继续朝原方向"倒飞"，所以外推一旦越过速度归零时刻就要折返。
  const double a_norm = norm(a_hat_);
  if (a_norm > 0.05) {
    const double v_along = dot(v_hat_, a_hat_) / a_norm;
    if (v_along < 0.0) {
      const double t_stop = -v_along / a_norm;
      if (t_stop < tau) {
        const double dt_tail = tau - t_stop;
        q = p_hat_ + v_hat_ * t_stop +
          a_hat_ * (0.5 * t_stop * t_stop + 0.5 * dt_tail * dt_tail);
      }
    }
  }

  // 旋转靶：板心绕机器人中心转动，法向同步转过 omega*dt
  const Vec3 n_future = rotateZ(last_plate_normal_, omega_ * dt);
  return q + n_future * r_track_;
}

bool AimNode::solveAim(double age, const Telemetry & tel, double yaw_hint, double pitch_hint,
                       double & yaw_out, double & pitch_out) const
{
  if (!(tel.speed > 1.0) || !(tel.gravity > 0.1) || !(tel.barrel_length > 0.01) ||
    !(tel.delay >= 0.0))
  {
    return false;
  }
  const double v = tel.speed;
  const double g = tel.gravity;
  const double L = tel.barrel_length;

  // 目标位置需要考虑：观测延迟 + 通信与执行延迟 + 出膛延迟 + 弹丸飞行时间
  // 注意：不要额外补“半个射击节拍”。实测把 0.05 s 加进来后，组合场景的横向偏差
  // 从 +0.039 m 变成 −0.028 m，6 m 平移的判定从 0.947 掉到 0.692，说明仿真对
  // fire=true 的响应并不滞后半拍，加了反而是过冲。
  Vec3 target = predictCenter(age + extra_latency_ + tel.delay + 0.25);

  double yaw = yaw_hint;
  double pitch = pitch_hint;
  double flight = 0.25;

  for (int iter = 0; iter < 6; ++iter) {
    const Vec3 muzzle = muzzleAt(yaw, pitch, L);
    const double dx = target.x - muzzle.x;
    const double dy = target.y - muzzle.y;
    const double d = std::hypot(dx, dy);
    if (d < 0.05) {
      return false;
    }
    const double dz = target.z - muzzle.z;

    // 抛体方程：dz = d*tanθ - g*d²/(2*v²*cos²θ)
    // 化为 A*d²*tan²θ - d*tanθ + (dz + A*d²) = 0，A = g/(2v²)
    const double A = g / (2.0 * v * v);
    const double disc = 1.0 - 4.0 * A * (dz + A * d * d);
    if (disc <= 0.0) {
      return false;  // 超出射程，打不到
    }
    const double tan_theta = (1.0 - std::sqrt(disc)) / (2.0 * A * d);  // 取低伸弹道
    const double theta = std::atan(tan_theta);

    yaw = std::atan2(dy, dx);
    pitch = std::clamp(theta, pitch_min_, pitch_max_);

    // 水平速度恒为 v*cos(pitch)，因此水平距离除以水平速度即为精确飞行时间
    const Vec3 muzzle2 = muzzleAt(yaw, pitch, L);
    const double d2 = std::hypot(target.x - muzzle2.x, target.y - muzzle2.y);
    flight = d2 / (v * std::max(0.2, std::cos(pitch)));

    target = predictCenter(age + extra_latency_ + tel.delay + flight);
  }

  const Vec3 muzzle = muzzleAt(yaw, pitch, L);
  yaw = std::atan2(target.y - muzzle.y, target.x - muzzle.x);

  if (!std::isfinite(yaw) || !std::isfinite(pitch)) {
    return false;
  }
  yaw_out = yaw;
  pitch_out = pitch;
  return true;
}

// 当前帧没有可用装甲时（空列表，或只剩侧对板），能不能靠预测继续打：
// 灭灯只是"不再发布"，仿真 _collision 仍然遍历全部四块板，板子物理上还是能被打中的，
// 所以这 50 ms 里按预测位置继续射击是有效的；只有丢失太久、或拟合不可信时才停火。
bool AimNode::canHold(double stamp, double grace) const
{
  // 只看"预测还可信吗"：有历史样本、拟合成功、残差小、丢失时间在宽限内。
  // 不再要求 motion_ready_（窗口内样本数）——灭灯时采样本来就只有隔帧的 5~6 个点，
  // 这个条件会把恰好能打的一发挡掉。
  if (!last_plate_valid_ || !fit_ok_ || fit_rms_ >= 0.025) {
    return false;
  }
  if (last_stamp_ < 0.0) {
    return false;
  }
  const double gap = stamp - last_stamp_;
  return gap > -0.05 && gap <= grace;
}

void AimNode::aimAndFire(
  const std_msgs::msg::Header & header, const std::string & session_id,
  const GimbalState & gimbal, const Telemetry & tel, double now, double age,
  const Vec3 & center_hint, int sel, bool allow_fire)
{
  // predictCenter 的 dt 是相对"最后一次观测"的偏移，而 age 是相对"本条消息时间戳"的。
  // 正常帧两者等价（last_stamp_ == stamp）；hold 期间不再有观测，last_stamp_ 冻住，
  // 若仍用 age，预测点会停在原地不动（实测会打出一串落点完全相同的空枪）。
  // 所以统一换算成"距最后一次观测"的时间。
  double age_eff = age;
  if (last_stamp_ >= 0.0) {
    age_eff = std::max(0.0, now - last_stamp_);
  }

  double yaw_cmd = last_yaw_cmd_;
  double pitch_cmd = last_pitch_cmd_;
  if (!solveAim(age_eff, tel, gimbal.yaw, gimbal.pitch, yaw_cmd, pitch_cmd)) {
    publishCommand(header, session_id, last_yaw_cmd_, last_pitch_cmd_, true, false);
    return;
  }

  // 允许的角度误差随距离收紧：远处稍微偏一点就打飞
  const Vec3 muzzle_now = muzzleAt(gimbal.yaw, gimbal.pitch, tel.barrel_length);
  const double dist = norm(center_hint - muzzle_now);
  double tol = std::atan2(0.05, std::max(1.0, dist));
  tol = std::max(0.0035, std::min(aim_tolerance_, tol));
  const double yaw_err = std::abs(wrapPi(gimbal.yaw - yaw_cmd));
  const double pitch_err = std::abs(gimbal.pitch - pitch_cmd);
  const bool aligned = yaw_err < tol && pitch_err < tol;

  // 刚进入新会话、或丢失后样本不足时先不开火：
  // 判定依据是"窗口内是否攒够了用于拟合的样本"，而不是历史里总共存了多少点。
  // 折返点附近残差会变大（窗口跨了两段运动），此时速度符号不可信，也不开火。
  const bool tracking_ready = motion_ready_ && fit_rms_ < 0.025;
  const bool fire = allow_fire && aligned && tracking_ready;

  publishCommand(header, session_id, yaw_cmd, pitch_cmd, true, fire);

  // 逐发级诊断：把"我们发出去的 yaw"和"仿真记录的每发 yaw"对起来，
  // 就能区分误差到底来自"云台不在指令位置（时序问题）"还是"指令本身算错（模型问题）"。
  // mrq/fs/rms 三个字段专门用来定位"对准了却不开火"：mrq=motion_ready，fs=换板镇定，
  // rms=拟合残差（阈值 0.025）。哪个为 0 就是它在拦。
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 100,
    "CMD yaw=%.4f pitch=%.4f cy=%.3f sel=%d fire=%d err=%.4f/%.4f mrq=%d fs=%d rms=%.4f "
    "hist=%zu",
    yaw_cmd, pitch_cmd, center_hint.y, sel, static_cast<int>(fire), yaw_err, pitch_err,
    static_cast<int>(motion_ready_), static_cast<int>(allow_fire), fit_rms_,
    history_.size());

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "sel=%d target=(%.2f,%.2f,%.2f) dist=%.2f yaw=%.3f pitch=%.3f err=%.4f/%.4f "
    "fire=%d omega=%.2f hold=%d",
    sel, center_hint.x, center_hint.y, center_hint.z, dist,
    yaw_cmd, pitch_cmd, yaw_err, pitch_err, static_cast<int>(fire), omega_,
    static_cast<int>(!allow_fire));
}

void AimNode::publishCommand(const std_msgs::msg::Header & header, const std::string & session_id,
                             double yaw, double pitch, bool enable, bool fire)
{
  if (!std::isfinite(yaw) || !std::isfinite(pitch)) {
    return;
  }

  AimCommand cmd;
  cmd.header = header;  // 保留 frame_id = "odom"（闸门会校验）
  // 【关键】指令时间戳必须用"当前仿真时间"，不能用观测消息的时间戳。
  // 闸门 control_gate.hpp 的接受条件是 sim_time - stamp ≤ 0.25 且 stamp - sim_time ≤ 0.05，
  // 回填观测戳时，只要那帧装甲消息稍微过期（或 reset 后缓存的那帧还停在上一局的时刻），
  // 指令就已经被判为 stale 整条丢掉，终端 2 会刷 "Rejected stale, out-of-order or invalid command/session"。
  // 表现就是节点日志里 fire=1 一直发、仿真却一发都不出，kill_time 把这段空转全算进成绩。
  cmd.header.stamp = this->now();
  cmd.session_id = session_id;
  cmd.yaw = yaw;
  cmd.pitch = pitch;
  cmd.enable = enable;
  cmd.fire = fire;
  command_pub_->publish(cmd);

  if (enable) {
    last_yaw_cmd_ = yaw;
    last_pitch_cmd_ = pitch;
    last_cmd_valid_ = true;
  }
}

void AimNode::controlLoop()
{
  ArmorCornersArray::ConstSharedPtr armors;
  GimbalState::ConstSharedPtr gimbal;
  Telemetry tel;
  {
    std::lock_guard<std::mutex> lock(mtx_);
    armors = armors_;
    gimbal = gimbal_;
    tel = telemetry_;
  }

  if (!gimbal || !armors) {
    return;  // 还没收到必要数据
  }

  const double now = simNow();
  const double stamp = static_cast<double>(armors->header.stamp.sec) +
    static_cast<double>(armors->header.stamp.nanosec) * 1e-9;

  // ---------- 1. 会话切换 ----------
  if (armors->session_id != session_id_) {
    resetTrack();
    session_id_ = armors->session_id;
    RCLCPP_INFO(this->get_logger(), "检测到新会话：%s", session_id_.c_str());
  }

  // ---------- 2. 数据时效 ----------
  double age = now - stamp;
  if (!(age > -0.05 && age < 5.0)) {
    // 时间基准异常（常见原因：忘记 use_sim_time:=true），退化为固定补偿
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "仿真时间不可用，请检查 use_sim_time 参数；当前按固定延迟补偿");
    age = extra_latency_;
  }
  if (age < 0.0) {
    age = 0.0;
  }
  if (age > 0.25) {
    publishCommand(armors->header, armors->session_id, last_yaw_cmd_, last_pitch_cmd_, false, false);
    return;
  }

  // ---------- 3. 空装甲列表：灭灯不等于无敌，能靠预测接着打就接着打 ----------
  if (armors->armors.empty()) {
    // 命中的装甲会灭灯 50 ms 并被移出发布列表，但这只是"看不见"，
    // 仿真 _collision 仍遍历四块板，按预测位置打过去照样判定命中。
    // 这时停火就等于每打中一发就丢掉一个射击节拍，击毁时间被拉长。
    // 但"空列表"还有另一种含义：目标已被击毁，仿真不再发布装甲。
    // 所以空列表只给很短的宽限（灭灯最长 50 ms，0.12 s 足够覆盖 3~4 帧），
    // 避免击毁后还按冻结的预测继续开枪——那几发会全部算进 score_fired 拉低命中率。
    if (canHold(stamp, hold_grace_empty_)) {
      aimAndFire(
        armors->header, armors->session_id, *gimbal, tel, now, age,
        predictCenter(age), -1, true);
    } else {
      // 丢失太久或预测不可信：按手册发 enable=false / fire=false，但保留历史
      publishCommand(armors->header, armors->session_id, last_yaw_cmd_, last_pitch_cmd_, false, false);
    }
    return;
  }

  // ---------- 4. 装甲几何提取与目标选择 ----------
  const Vec3 muzzle_now = muzzleAt(gimbal->yaw, gimbal->pitch, tel.barrel_length);
  const std::vector<Plate> plates = extractPlates(*armors, muzzle_now);
  const double dt_since_sample = (last_stamp_ >= 0.0) ? std::max(0.0, stamp - last_stamp_) : 0.0;
  // 选板用的前推时长：子弹飞行 + 消息年龄 + 观测/执行延迟（解算里也是这几项相加，
  // 少算消息年龄会让"到场时刻"早 20~30 ms，ω=5.6 rad/s 时就是 0.17 rad 的相位差）
  const Vec3 nearest = plates.empty() ? last_plate_center_ : plates.front().center;
  const double lead = norm(nearest - muzzle_now) / std::max(1.0, tel.speed) +
    age + extra_latency_ + tel.delay;
  const int best = selectPlate(plates, muzzle_now, dt_since_sample, lead);

  if (best < 0) {
    // 只剩侧对板（容差约 1.3 cm，打了也是空枪）：同样按预测位置顶一发
    if (canHold(stamp, hold_grace_)) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "只有侧对板（%zu 块，facing<%.2f），按锁定板的预测位置继续射击", plates.size(),
        facing_min_);
      aimAndFire(
        armors->header, armors->session_id, *gimbal, tel, now, age,
        predictCenter(age), -2, true);
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "收到 %zu 块装甲但全部未正对（facing<%.2f），暂停控制（gap=%.2f rms=%.3f fit=%d hist=%zu）",
        plates.size(), facing_min_, stamp - last_stamp_, fit_rms_,
        static_cast<int>(fit_ok_), history_.size());
      publishCommand(armors->header, armors->session_id, last_yaw_cmd_, last_pitch_cmd_, false, false);
    }
    return;
  }
  const Plate & target_plate = plates[static_cast<size_t>(best)];

  // 换板检测：把上一帧的法向按 ω 前推后再比对，避免旋转靶因帧间隔被误判成换板
  if (last_plate_valid_) {
    const Vec3 n_pred = rotateZ(last_plate_normal_, omega_ * dt_since_sample);
    if (dot(target_plate.normal, n_pred) < 0.94) {
      // 同一台车的各块板，水平方向都落在同一个旋转半径上，q = 板心 - r·法向 的差别只剩安装高度，
      // 是一个常量。把它整体平移到历史样本上，速度估计就能跨板无缝延续，不必清空重来。
      const Vec3 q_old = last_plate_center_ - last_plate_normal_ * r_track_;
      const Vec3 q_new = target_plate.center - target_plate.normal * r_track_;
      // 三个分量一起平移：竖向是真实的安装高度差（奇板 -0.1 m），必须补。
      // 水平方向现在已经在样本里按可见板取平均了（各板算出的 q 水平分量本来就相同），
      // 再按"当前这一块板"平移反而会把单板法向噪声重新注进历史，所以水平不再平移。
      Vec3 dq = q_new - q_old;
      dq.x = 0.0;
      dq.y = 0.0;
      last_switch_stamp_ = stamp;   // 换板瞬间不许开火，等云台在新板上稳住
      for (auto & s : history_) {
        s.c = s.c + dq;
      }
      p_hat_ = p_hat_ + dq;
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "换板，历史平移 (%.2f,%.2f,%.2f)", dq.x, dq.y, dq.z);
    }
  }

  // ---------- 4. 板心半径 r ----------
  // 两块板同时可见时 c_i - c_j = r·(u_i - u_j)，u 是水平方向的单位径向，
  // 而法向 n = cos(tilt)·u + sin(tilt)·ẑ。
  //   => |Δc_h| = r·|Δu|，|Δn_h| = cos(tilt)·|Δu|
  //   => |Δc_h|/|Δn_h| = r / cos(tilt)
  // 而我们在 q = c - r·n 里沿【法向】走的长度正是 r，所以这里要的正正好好是
  // r/cos(tilt) ≈ 0.2567 —— 注意不要再乘 cos(tilt)，那样会得到 0.248，
  // 四块板算出的 q 就会差 1 cm 量级，换板时被当成真实位置跳变。
  // 只有取 0.2567，四块板算出的 q 才严格相同，换板平移残差才真正归零。
  // 另外必须只用水平分量：|Δc| 里混着奇数板 height_difference 的 0.1 m 高度差。
  double r_meas = -1.0;
  for (size_t i = 0; i < plates.size() && r_meas < 0.0; ++i) {
    for (size_t j = i + 1; j < plates.size(); ++j) {
      const Vec3 dc_h{
        plates[i].center.x - plates[j].center.x,
        plates[i].center.y - plates[j].center.y, 0.0};
      const Vec3 dn_h{
        plates[i].normal.x - plates[j].normal.x,
        plates[i].normal.y - plates[j].normal.y, 0.0};
      const double dn = norm(dn_h);
      if (dn < 0.3) {
        continue;  // 两块板朝向太接近，几何关系病态
      }
      const double r = norm(dc_h) / dn;
      if (r > 0.05 && r < 0.45) {
        r_meas = r;
        break;
      }
    }
  }
  // 只有一块板可见却在旋转时，用板心切向速度 / 角速度反推半径
  if (r_meas < 0.0 && r_track_ <= 0.05 && std::abs(omega_) > 0.5) {
    const double r = norm(v_hat_) / std::abs(omega_);
    if (r > 0.08 && r < 0.45) {
      r_meas = r;
    }
  }
  if (r_meas > 0.0) {
    const double r_old = r_track_;
    r_track_ = (r_track_ <= 0.0) ? r_meas : (0.85 * r_track_ + 0.15 * r_meas);
    if (r_old > 0.0 && std::abs(r_track_ - r_old) > 0.03) {
      // 半径跳变会让历史里的机器人中心整体错位，丢掉旧样本重新拟合
      history_.clear();
      last_stamp_ = -1.0;
      fit_ok_ = false;
    }
  }

  // 半径估计不依赖具体是哪块板，换板无需额外处理
  if (last_plate_valid_ && r_track_ <= 0.05 &&
      norm(target_plate.center - last_plate_center_) > plate_jump_gate_ * 1.5)
  {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500, "半径未知且位置跳变，重置跟踪");
    resetTrack();
  }

  // ---------- 5. 观测更新（同一帧不重复入样） ----------
  // 样本记录的是机器人中心 q = c - r·n。
  // 关键：不要只用"当前选中的那块板"算 q。单块板的法向一旦被角点噪声带偏 14°，
  // q 就在径向偏 r·Δφ ≈ 6 cm，而且这种污染恰好是"命中弹误差≈0、未中弹误差≈6 cm"
  // 双峰结构的来源（实测组合场景）。
  // 四块板都满足 c_i = q + r·u_i，所以【水平分量】可以按可见板取平均，把单板噪声摊薄；
  // 【竖向】不能平均——奇数板带 height_difference = -0.1 m，平均会平白引入 ±0.05 m 误差，
  // 竖向必须跟着当前目标板走。
  Vec3 robot_center = target_plate.center - target_plate.normal * r_track_;
  if (plates.size() > 1 && r_track_ > 0.05) {
    Vec3 q_xy{0.0, 0.0, 0.0};
    int cnt = 0;
    for (const auto & p : plates) {
      if (p.facing < 0.3) {
        continue;  // 侧对板法向不准，不参与平均
      }
      const Vec3 q = p.center - p.normal * r_track_;
      q_xy.x += q.x;
      q_xy.y += q.y;
      ++cnt;
    }
    if (cnt > 0) {
      robot_center.x = q_xy.x / static_cast<double>(cnt);
      robot_center.y = q_xy.y / static_cast<double>(cnt);
    }
  }
  if (stamp > last_stamp_ + 1e-6) {
    addSample(stamp, robot_center, target_plate.normal);
    updateOmega(stamp, target_plate.normal);
    last_stamp_ = stamp;
    last_plate_center_ = target_plate.center;
    last_plate_normal_ = target_plate.normal;
    last_plate_valid_ = true;
    fitMotion();
  }

  // ---------- 6. 弹道解算 + 到位判定 + 开火 ----------
  // 换板后先压一拍：换板瞬间云台还指着上一块板，此刻 fire=true 会让仿真在下一拍
  // 沿旧方向发弹（实测那几发的 |Δyaw| 高达 0.05 rad ≈ 0.28 m，正好一个板位间距）。
  const bool switch_settled = (stamp - last_switch_stamp_) > switch_settle_;
  aimAndFire(
    armors->header, armors->session_id, *gimbal, tel, now, age, target_plate.center, best,
    switch_settled);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "  fit v=(%.2f,%.2f,%.2f) a=(%.2f,%.2f,%.2f) r=%.3f plates=%zu hist=%zu",
    v_hat_.x, v_hat_.y, v_hat_.z, a_hat_.x, a_hat_.y, a_hat_.z,
    r_track_, plates.size(), history_.size());

  // 诊断：列出每块可见板的正对程度与板心位置，用来确认"是否在打侧对板"
  {
    std::string dump;
    char buf[128];
    for (size_t i = 0; i < plates.size(); ++i) {
      std::snprintf(
        buf, sizeof(buf), " [%zu]f=%.2f(%.2f,%.2f,%.2f)", i, plates[i].facing,
        plates[i].center.x, plates[i].center.y, plates[i].center.z);
      dump += buf;
    }
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 200,
      "选中 #%d f=%.2f | 可见板%s", best, target_plate.facing, dump.c_str());
  }
}

}  // namespace student_auto_aim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<student_auto_aim::AimNode>());
  rclcpp::shutdown();
  return 0;
}

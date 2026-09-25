/**
 * uav_png_intercept — 比例导引法 (PNG) 无人机拦截节点
 *
 * 控制: px4_1（拦截机）  拦截: px4_2（目标，由 uav_target_sim 控制）
 *
 * 坐标系方案：
 *   ► 订阅 SensorGps（非 VehicleLocalPosition.ref_lat），可靠性更高
 *   ► 以 px4_1 的第一个 GPS 读数为 LocalCartesian 原点
 *   ► 将 px4_2 的 GPS 实时转换到该坐标系下 → target_pos_（NED）
 *   ► 自身位置同样用 GPS→LocalCartesian 转换 → self_pos_（NED）
 *   ► 速度来自各自的 VehicleLocalPosition（NED 方向一致，无需偏移）
 *
 * PNG 算法（与现有 uav_ibvs_control/los_control.cpp 完全一致的角度更新公式）：
 *   LOS_angle_v = atan2(Rz, sqrt(Rx²+Ry²))   仰角（NED Down 为正）
 *   LOS_angle_z = atan2(Rx, Ry)               方位角（NED North 为 0）
 *   d_v_angle_v = N × (LOS_v - last_LOS_v) + last_v_angle_v
 *   d_v_angle_z = N × (LOS_z - last_LOS_z) + last_v_angle_z
 *   v_cmd = [cos(d_v_v)·sin(d_v_z), cos(d_v_v)·cos(d_v_z), sin(d_v_v)] × speed
 */

#include "uav_png_intercept/png_control.hpp"
#include <iostream>

// ============================================================
//  构造函数
// ============================================================
PngInterceptor::PngInterceptor() : Node("png_interceptor")
{
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // ---------- 发布者 ----------
    offboard_pub_ = create_publisher<OffboardControlMode>(
        "/px4_1/fmu/in/offboard_control_mode", 10);
    traj_pub_ = create_publisher<TrajectorySetpoint>(
        "/px4_1/fmu/in/trajectory_setpoint", 10);
    cmd_pub_ = create_publisher<VehicleCommand>(
        "/px4_1/fmu/in/vehicle_command", 10);

    // ---------- 订阅：自身 GPS（px4_1）→ 建立坐标系 + 自身位置 ----------
    self_gps_sub_ = create_subscription<SensorGps>(
        "/px4_1/fmu/out/vehicle_gps_position", qos,
        [this](const SensorGps::SharedPtr msg) {
            // 第一个 GPS 读数：初始化 LocalCartesian 参考原点
            if (!lc_initialized_) {
                lc_ = GeographicLib::LocalCartesian(
                    msg->latitude_deg, msg->longitude_deg, msg->altitude_msl_m);
                lc_initialized_ = true;
                RCLCPP_INFO(get_logger(),
                    "[px4_1 GPS] 坐标系原点已初始化: lat=%.7f lon=%.7f alt=%.2f",
                    msg->latitude_deg, msg->longitude_deg, msg->altitude_msl_m);
            }
            // 自身位置：GPS → LocalCartesian ENU → NED
            double enu_e, enu_n, enu_u;
            lc_.Forward(msg->latitude_deg, msg->longitude_deg, msg->altitude_msl_m,
                        enu_e, enu_n, enu_u);
            self_pos_ = {(float)enu_n, (float)enu_e, (float)(-enu_u)};
            self_pos_ok_ = true;
        });

    // ---------- 订阅：目标 GPS（px4_2）→ 实时位置 ----------
    target_gps_sub_ = create_subscription<SensorGps>(
        "/px4_2/fmu/out/vehicle_gps_position", qos,
        [this](const SensorGps::SharedPtr msg) {
            if (!lc_initialized_) return;  // 等待自身坐标系建立
            double enu_e, enu_n, enu_u;
            lc_.Forward(msg->latitude_deg, msg->longitude_deg, msg->altitude_msl_m,
                        enu_e, enu_n, enu_u);
            target_pos_ = {(float)enu_n, (float)enu_e, (float)(-enu_u)};
            target_pos_ok_ = true;
        });

    // ---------- 订阅：自身速度（px4_1 VehicleLocalPosition）----------
    self_vel_sub_ = create_subscription<VehicleLocalPosition>(
        "/px4_1/fmu/out/vehicle_local_position", qos,
        [this](const VehicleLocalPosition::SharedPtr msg) {
            self_vel_ = {msg->vx, msg->vy, msg->vz};
            local_z_ = msg->z;
        });

    // ---------- 订阅：目标速度（px4_2 VehicleLocalPosition）----------
    // 速度方向（NED）在两架无人机中是一致的，无需坐标系转换
    target_vel_sub_ = create_subscription<VehicleLocalPosition>(
        "/px4_2/fmu/out/vehicle_local_position", qos,
        [this](const VehicleLocalPosition::SharedPtr msg) {
            target_vel_ = {msg->vx, msg->vy, msg->vz};
        });

    // ---------- 订阅：飞行状态 ----------
    status_sub_ = create_subscription<VehicleStatus>(
        "/px4_1/fmu/out/vehicle_status", qos,
        [this](const VehicleStatus::SharedPtr msg) {
            nav_state_    = msg->nav_state;
            arming_state_ = msg->arming_state;
            offboard_active_ =
                (msg->nav_state == msg->NAVIGATION_STATE_OFFBOARD);
            status_received_ = true;
        });

    // ---------- 50Hz 控制定时器 ----------
    timer_ = create_wall_timer(30ms,
        std::bind(&PngInterceptor::control_loop, this));

    RCLCPP_INFO(get_logger(), "PNG Interceptor 节点已启动，等待 GPS 数据...");
}

// ============================================================
//  比例导引核心（与 uav_ibvs_control/los_control.cpp 公式一致）
//
//  LOS 角度定义（NED 坐标系）：
//    LOS_angle_v = atan2(Rz, sqrt(Rx²+Ry²))  仰角（D轴向下为正）
//    LOS_angle_z = atan2(Rx, Ry)              方位角（北为0，顺时针）
//
//  PNG 更新：
//    d_v_angle_v = N × (LOS_v - last_LOS_v) + last_v_angle_v
//    d_v_angle_z = N × (LOS_z - last_LOS_z) + last_v_angle_z
//
//  期望速度：
//    vx = cos(d_v_v) × sin(d_v_z)
//    vy = cos(d_v_v) × cos(d_v_z)
//    vz = sin(d_v_v)
// ============================================================
Eigen::Vector3f PngInterceptor::compute_png_velocity(
    const Eigen::Vector3f& target_pos_ned)
{
    Eigen::Vector3f R = target_pos_ned - self_pos_;
    float r = R.norm();

    if (r < 0.3f) {
        return R.normalized() * intercept_speed_;
    }

    // ---- 计算当前 LOS 角度（水平面方位角 + 仰角）----
    float rxy = sqrtf(R(0)*R(0) + R(1)*R(1));
    float LOS_angle_v = atan2f(R(2), rxy);          // 仰角（NED Down 为正）
    float LOS_angle_z = atan2f(R(0), R(1));          // 方位角（NED：北=0，顺时针）
    target_yaw_ = atan2f(R(1), R(0));               // PX4 yaw = atan2(East, North)

    // ---- 首次进入：用 LOS 方向初始化，不使用当前飞行速度 ----
    // （避免 TAKEOFF 残余爬升速度污染初始 PNG 状态）
    if (!png_initialized_) {
        last_LOS_angle_v_ = LOS_angle_v;
        last_LOS_angle_z_ = LOS_angle_z;
        d_v_angle_v_      = LOS_angle_v;
        d_v_angle_z_      = LOS_angle_z;
        last_v_angle_v_   = LOS_angle_v;
        last_v_angle_z_   = LOS_angle_z;
        png_initialized_  = true;
        RCLCPP_INFO(get_logger(), "PNG 已初始化，LOS: v=%.3f z=%.3f",
                    LOS_angle_v, LOS_angle_z);
    }

    float diff_LOS_v = LOS_angle_v - last_LOS_angle_v_;
    float diff_LOS_z = LOS_angle_z - last_LOS_angle_z_;

    // ---- 修复 Bug1：方位角穿越 ±π 导致 diff 跳变约 2π ----
    // atan2 结果范围 (-π, π]，相邻两帧之差超过 π 则说明发生了穿越
    while (diff_LOS_z >  (float)M_PI) diff_LOS_z -= 2.0f * (float)M_PI;
    while (diff_LOS_z < -(float)M_PI) diff_LOS_z += 2.0f * (float)M_PI;
    // 仰角一般不会穿越，但为安全起见也加上
    while (diff_LOS_v >  (float)M_PI) diff_LOS_v -= 2.0f * (float)M_PI;
    while (diff_LOS_v < -(float)M_PI) diff_LOS_v += 2.0f * (float)M_PI;

    // ---- LOS 角度变化足够大时执行 PNG 更新 ----
    if (fabsf(diff_LOS_v) > 0.005f || fabsf(diff_LOS_z) > 0.005f) {

        // 修复 Bug2：用「上一帧的期望速度角」作为 last_v_angle，
        // 而不是当前测量速度（测量速度在 TAKEOFF→INTERCEPT 切换初期
        // 含有大量残余爬升速度，会把 PNG 拉歪）
        last_v_angle_v_ = d_v_angle_v_;
        last_v_angle_z_ = d_v_angle_z_;

        // PNG 角度更新（与 los_control.cpp 公式完全一致）
        d_v_angle_v_ = N_ * diff_LOS_v + last_v_angle_v_;
        d_v_angle_z_ = N_ * diff_LOS_z + last_v_angle_z_;

        // 限制仰角范围，防止指向近垂直方向（±60°）
        d_v_angle_v_ = std::max(-(float)M_PI/3.0f,
                        std::min( (float)M_PI/3.0f, d_v_angle_v_));

        last_LOS_angle_v_ = LOS_angle_v;
        last_LOS_angle_z_ = LOS_angle_z;

        RCLCPP_INFO(get_logger(),
            "[PNG] dist=%.2f | LOS(v=%.3f z=%.3f) | dLOS(v=%.3f z=%.3f) | dV(v=%.3f z=%.3f)",
            r, LOS_angle_v, LOS_angle_z,
            diff_LOS_v, diff_LOS_z,
            d_v_angle_v_, d_v_angle_z_);
    }

    // ---- 水平速度分量（PNG 控制方位 + 仰角）----
    float d_v = self_vel_.norm() + 2.0f;   // 缓加速，避免大幅俯仰导致目标出视野
    d_v = std::min(d_v, intercept_speed_);
    d_v = std::max(d_v, 2.0f);

    float vx = cosf(d_v_angle_v_) * sinf(d_v_angle_z_) * d_v;  // North
    float vy = cosf(d_v_angle_v_) * cosf(d_v_angle_z_) * d_v;  // East

    // ---- 垂直速度分量 ----
    // 略高于目标 (0.5m)，补偿前飞低头导致的相机下倾
    float alt_offset = 0.25f;
    float alt_err = (target_pos_ned(2) - alt_offset) - self_pos_(2);
    float vz = std::max(-3.0f, std::min(3.0f, alt_err * 2.0f));

    return {vx, vy, vz};
}

// ============================================================
//  主控制循环 20Hz
// ============================================================
void PngInterceptor::control_loop()
{
    switch (state_) {
    case InterceptState::INIT:    handle_init();      break;
    case InterceptState::TAKEOFF: handle_takeoff();   break;
    case InterceptState::INTERCEPT: handle_intercept(); break;
    case InterceptState::DONE:
        publish_offboard_velocity_mode();
        publish_velocity_setpoint(0.0f, 0.0f, 0.0f, std::nanf(""));
        RCLCPP_INFO_ONCE(get_logger(), "★★★ 拦截完成，悬停中 ★★★");
        break;
    }
    save_data_to_csv();
}

// ============================================================
//  INIT：持续发心跳帧 + arm/offboard，2 秒后无条件进 TAKEOFF
//
//  关键修复：不依赖 offboard_active_ 作为转移条件。
//  PX4 需要先接收到连续的 OffboardControlMode 数据流（建议 2s+），
//  然后 arm + SET_MODE 才能生效。TAKEOFF 状态会继续重复发送这些命令
//  直到 PX4 真正切换到 Offboard 并起飞。
// ============================================================
void PngInterceptor::handle_init()
{
    // 始终发送 Offboard 心跳（PX4 要求进入 Offboard 前数据流必须存在）
    publish_offboard_position_mode();
    publish_position_setpoint(0.0f, 0.0f, takeoff_alt_, std::nanf(""));

    // 从第 1 帧起就持续发 arm + SET_MODE（与 uav_ibvs_control 一致）
    set_offboard_mode();
    arm();

    // 每 20 帧（1 秒）打印诊断信息
    if (init_counter_ % 20 == 0) {
        RCLCPP_INFO(get_logger(),
            "[INIT] cnt=%d | gps=%s | status_rx=%s | nav_state=%d | arming=%d | offboard=%s | target=%s",
            init_counter_,
            lc_initialized_  ? "YES" : "NO",
            status_received_ ? "YES" : "NO",
            nav_state_,
            arming_state_,
            offboard_active_ ? "YES" : "NO",
            target_pos_ok_   ? "YES" : "NO");
    }

    // GPS 就绪且等待 2 秒后，无条件切换到 TAKEOFF
    // （TAKEOFF 状态会继续发送 arm/offboard，直到 PX4 真正响应）
    if (lc_initialized_ && init_counter_ > 40) {
        state_ = InterceptState::TAKEOFF;
        RCLCPP_INFO(get_logger(),
            "→ INIT 结束，进入 TAKEOFF（当前 nav_state=%d arming=%d）",
            nav_state_, arming_state_);
    }

    if (init_counter_ < 10000) init_counter_++;
}

// ============================================================
//  TAKEOFF：位置控制爬升到目标高度
// ============================================================
void PngInterceptor::handle_takeoff()
{
    // 持续发送，保持 offboard 模式
    set_offboard_mode();
    arm();
    publish_offboard_position_mode();

    // 机头指向目标（与 INTERCEPT 阶段一致）
    float yaw = std::nanf("");
    if (target_pos_ok_ && self_pos_ok_) {
        Eigen::Vector3f R = target_pos_ - self_pos_;
        yaw = atan2f(R(1), R(0));  // yaw = atan2(East, North)
    }
    publish_position_setpoint(0.0f, 0.0f, takeoff_alt_, yaw);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[TAKEOFF] 当前高度 z=%.2f m (目标 %.2f m, 差 %.2f m)",
        local_z_, takeoff_alt_, std::fabs(local_z_ - takeoff_alt_));

    // 高度到位（误差 < 0.5m）且目标数据就绪
    if (self_pos_ok_ &&
        std::fabs(local_z_ - takeoff_alt_) < 0.5f &&
        target_pos_ok_)
    {
        state_ = InterceptState::INTERCEPT;
        RCLCPP_INFO(get_logger(), "→ 高度到位，切换到 INTERCEPT");
    }
}

// ============================================================
//  INTERCEPT：PNG 导引拦截
// ============================================================
void PngInterceptor::handle_intercept()
{
    set_offboard_mode();
    arm();

    if (!target_pos_ok_ || !self_pos_ok_) {
        publish_offboard_velocity_mode();
        publish_velocity_setpoint(0.0f, 0.0f, 0.0f, std::nanf(""));
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "[INTERCEPT] 等待位置数据... self=%s target=%s",
            self_pos_ok_ ? "OK" : "NO",
            target_pos_ok_ ? "OK" : "NO");
        return;
    }

    Eigen::Vector3f R = target_pos_ - self_pos_;
    float dist = R.norm();

    // 打印关键调试信息（每帧）
    RCLCPP_INFO(get_logger(),
        "[INTERCEPT] dist=%.2f m | T(%.2f,%.2f,%.2f) S(%.2f,%.2f,%.2f) Vself(%.1f,%.1f,%.1f)",
        dist,
        target_pos_(0), target_pos_(1), target_pos_(2),
        self_pos_(0),   self_pos_(1),   self_pos_(2),
        self_vel_(0),   self_vel_(1),   self_vel_(2));

    // 判断拦截成功
    if (dist < hit_radius_) {
        state_ = InterceptState::DONE;
        RCLCPP_WARN(get_logger(), "★★★ 目标命中！距离 = %.3f m ★★★", dist);
        return;
    }

    Eigen::Vector3f v_cmd = compute_png_velocity(target_pos_);
    publish_offboard_velocity_mode();
    publish_velocity_setpoint(v_cmd(0), v_cmd(1), v_cmd(2), target_yaw_);
}

// ============================================================
//  PX4 话题发布
// ============================================================
void PngInterceptor::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

void PngInterceptor::set_offboard_mode()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
}

void PngInterceptor::publish_offboard_position_mode()
{
    OffboardControlMode msg{};
    msg.position  = true;
    msg.velocity  = false;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    offboard_pub_->publish(msg);
}

void PngInterceptor::publish_offboard_velocity_mode()
{
    OffboardControlMode msg{};
    msg.position  = false;
    msg.velocity  = true;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    offboard_pub_->publish(msg);
}

void PngInterceptor::publish_position_setpoint(float x, float y, float z, float yaw)
{
    TrajectorySetpoint msg{};
    msg.position  = {x, y, z};
    msg.yaw       = yaw;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    traj_pub_->publish(msg);
}

void PngInterceptor::publish_velocity_setpoint(float vx, float vy, float vz, float yaw)
{
    TrajectorySetpoint msg{};
    msg.position  = {std::nanf(""), std::nanf(""), std::nanf("")};
    msg.velocity  = {vx, vy, vz};
    msg.yaw       = yaw;              // 机头指向目标
    msg.yawspeed  = std::nanf("");
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    traj_pub_->publish(msg);
}

void PngInterceptor::publish_vehicle_command(uint16_t command, float p1, float p2)
{
    VehicleCommand msg{};
    msg.param1           = p1;
    msg.param2           = p2;
    msg.command          = command;
    msg.target_system    = 2;   // px4_1 的 MAVLink system ID（Instance 1 → DDS key 2）
    msg.target_component = 1;
    msg.source_system    = 1;
    msg.source_component = 1;
    msg.from_external    = true;
    msg.timestamp        = get_clock()->now().nanoseconds() / 1000;
    cmd_pub_->publish(msg);
}

void PngInterceptor::save_data_to_csv() {
    if (!csv_file_.is_open()) {
        // 使用绝对路径，防止找不到文件
        csv_file_.open("/home/verser/ros2_ws/src/uav_png_intercept/intercept_data.csv", std::ios::out);
        csv_file_ << "time,s_x,s_y,s_z,t_x,t_y,t_z,dist\n";
    }

    // 假设 self_pos_ 和 target_pos_ 已经是转换后的 ENU 坐标
    double distance = (target_pos_ - self_pos_).norm();
    double ros_time = this->now().seconds();

    csv_file_ << std::fixed << std::setprecision(4)
              << ros_time << ","
              << self_pos_(0) << "," << self_pos_(1) << "," << -self_pos_(2) << "," // 转为 Up 为正
              << target_pos_(0) << "," << target_pos_(1) << "," << -target_pos_(2) << ","
              << distance << "\n";
    

}

// ============================================================
//  main
// ============================================================
int main(int argc, char* argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PngInterceptor>());
    rclcpp::shutdown();
    return 0;
}

/**
 * rl_guidance_node.cpp
 *
 * 基于强化学习 (GRU 策略) 的视觉制导拦截节点 — C++ 实现
 * 模型推理使用 ONNX Runtime
 * ── uav_vision_png 的可替换实现
 */

#include "uav_rl_guidance/rl_guidance_node.hpp"

#include <cmath>
#include <cstdio>
#include <chrono>

// PX4 QoS profile
#include <rmw/qos_profiles.h>

// ============================================================
//  常量
// ============================================================
static constexpr int WATCHDOG_LATCH = 20;   // 触发后保持 PNG 回退的帧数 (20Hz → 1s)
static constexpr float kMPerDegLatF = 111320.0f;

// R_c2b 静态常量定义（Cx→By, Cy→Bz, Cz→Bx）
const Eigen::Matrix3f RLGuidanceNode::R_c2b_ =
    (Eigen::Matrix3f() << 0, 0, 1,
                          1, 0, 0,
                          0, 1, 0).finished();

// 特征构造常量
static constexpr float _V_NORM       = 8.0f;
static constexpr float _LOS_RATE_NORM = 2.0f;
static constexpr float _LOGW_REF     = 2.995732273553991f;  // ln(20)
static constexpr float _AGE_MAX      = 2.0f;
static constexpr float _Z_NORM       = 20.0f;

inline float clamp(float x, float lo, float hi) {
    return std::max(lo, std::min(hi, x));
}

// ============================================================
//  几何辅助函数
// ============================================================

float RLGuidanceNode::wrap_pi(float angle)
{
    while (angle > M_PI)  angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

void RLGuidanceNode::quat_to_euler(float q0, float q1, float q2, float q3,
                                   float& roll, float& pitch, float& yaw)
{
    // PX4 quaternion (w, x, y, z) → ZYX Euler
    roll  = std::atan2(2.0f * (q0 * q1 + q2 * q3),
                       1.0f - 2.0f * (q1 * q1 + q2 * q2));
    float s2 = 2.0f * (q0 * q2 - q3 * q1);
    s2 = clamp(s2, -1.0f, 1.0f);
    pitch = std::asin(s2);
    yaw   = std::atan2(2.0f * (q0 * q3 + q1 * q2),
                       1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

Eigen::Matrix3f RLGuidanceNode::euler_to_r_b2n(float roll, float pitch, float yaw)
{
    // ZYX: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    Eigen::AngleAxisf Ryaw(yaw, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf Rpitch(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf Rroll(roll, Eigen::Vector3f::UnitX());
    return (Ryaw * Rpitch * Rroll).toRotationMatrix();
}

void RLGuidanceNode::pixel_to_los(float ex, float ey, float focal,
                                  float roll, float pitch, float yaw,
                                  float& los_v, float& los_z)
{
    Eigen::Vector3f nt_cam(ex, ey, focal);
    Eigen::Matrix3f R_b2n = euler_to_r_b2n(roll, pitch, yaw);
    Eigen::Vector3f nt_ned = R_b2n * R_c2b_ * nt_cam;
    float rxy = std::sqrt(nt_ned(0) * nt_ned(0) + nt_ned(1) * nt_ned(1));
    los_v = std::atan2(nt_ned(2), rxy);                     // elevation
    los_z = std::atan2(nt_ned(0), nt_ned(1));               // azimuth (North=0)
}

void RLGuidanceNode::angles_to_velocity(float v_angle_v, float v_angle_z, float speed,
                                        float& vx, float& vy, float& vz)
{
    vx = std::cos(v_angle_v) * std::sin(v_angle_z) * speed;  // North
    vy = std::cos(v_angle_v) * std::cos(v_angle_z) * speed;  // East
    vz = std::sin(v_angle_v) * speed;                         // Down
}

// ============================================================
//  策略推理 (ONNX Runtime)
// ============================================================

std::pair<std::array<float, 4>, std::array<float, 128>>
RLGuidanceNode::policy_inference(
    const std::array<float, 15>& obs, const std::array<float, 128>& h)
{
    // 输入 shape: obs [1, 15], h_in [1, 1, 128]
    std::array<int64_t, 2> obs_shape = {1, 15};
    std::array<int64_t, 3> h_shape   = {1, 1, 128};

    auto obs_tensor = Ort::Value::CreateTensor<float>(
        ort_memory_info_,
        const_cast<float*>(obs.data()), obs.size(),
        obs_shape.data(), obs_shape.size());

    auto h_tensor = Ort::Value::CreateTensor<float>(
        ort_memory_info_,
        const_cast<float*>(h.data()), h.size(),
        h_shape.data(), h_shape.size());

    const char* input_names[]  = {ort_input_name_obs_.c_str(),
                                  ort_input_name_h_.c_str()};
    const char* output_names[] = {ort_output_name_action_.c_str(),
                                  ort_output_name_h_.c_str()};
    Ort::Value inputs[] = {std::move(obs_tensor), std::move(h_tensor)};

    auto outputs = ort_session_->Run(Ort::RunOptions{nullptr},
        input_names, inputs, 2, output_names, 2);

    // 输出: action [1, 4], h_out [1, 1, 128]
    const float* action_data = outputs[0].GetTensorData<float>();
    const float* h_data      = outputs[1].GetTensorData<float>();

    std::array<float, 4>   action;
    std::array<float, 128> h_new;
    std::memcpy(action.data(), action_data, 4 * sizeof(float));
    std::memcpy(h_new.data(), h_data, 128 * sizeof(float));

    return {action, h_new};
}

// ============================================================
//  特征构造 (FeatureBuilder — 1:1 对应 guidance_rl/features.py)
// ============================================================

std::array<float, 15> RLGuidanceNode::build_features(
    const std::optional<std::tuple<int, int, int, int>>& det,
    float roll, float pitch, float yaw,
    float vx, float vy, float vz, float local_z, float dt)
{
    float dlos_v = 0, dlos_z = 0, dlog_w = 0;
    bool valid = det.has_value();

    if (valid) {
        auto [x, y, w, h] = *det;
        w = std::max(w, 1); h = std::max(h, 1);
        feat_ex_ = (x + w / 2.0f) - image_width_ / 2.0f;
        feat_ey_ = (y + h / 2.0f) - image_height_ / 2.0f;

        float los_v, los_z;
        pixel_to_los(feat_ex_, feat_ey_, focal_length_, roll, pitch, yaw, los_v, los_z);
        float log_w = std::log(static_cast<float>(w));
        float log_h = std::log(static_cast<float>(h));

        if (has_los_) {
            dlos_v = wrap_pi(los_v - feat_los_v_) / dt;
            dlos_z = wrap_pi(los_z - feat_los_z_) / dt;
            dlog_w = (log_w - log_w_) / dt;
        }

        feat_los_v_ = los_v;
        feat_los_z_ = los_z;
        log_w_ = log_w;
        log_h_ = log_h;
        has_los_ = true;
        feat_age_ = 0;
    } else {
        feat_age_ += dt;
    }

    float v_norm = std::sqrt(vx * vx + vy * vy + vz * vz);

    return {{
        feat_los_v_ / (static_cast<float>(M_PI) / 4.0f),         // 0: 仰角
        std::sin(feat_los_z_),                                     // 1: sin(方位)
        std::cos(feat_los_z_),                                     // 2: cos(方位)
        clamp(dlos_v / _LOS_RATE_NORM, -2.0f, 2.0f),             // 3: LOS 仰角率
        clamp(dlos_z / _LOS_RATE_NORM, -2.0f, 2.0f),             // 4: LOS 方位率
        (log_w_ - _LOGW_REF) / 2.0f,                              // 5: log(w) 归一化
        (log_h_ - _LOGW_REF) / 2.0f,                              // 6: log(h) 归一化
        clamp(dlog_w, -2.0f, 2.0f),                               // 7: 尺寸变化率
        valid ? 1.0f : 0.0f,                                      // 8: 有效帧标志
        std::min(feat_age_, _AGE_MAX) / _AGE_MAX,                  // 9: 丢失时间
        vx / _V_NORM,                                              // 10: 自身 vx
        vy / _V_NORM,                                              // 11: 自身 vy
        vz / _V_NORM,                                              // 12: 自身 vz
        v_norm / _V_NORM,                                          // 13: 自身速率
        -local_z / _Z_NORM,                                        // 14: 高度
    }};
}

// ============================================================
//  动作解码 (decode_action — 1:1 对应 guidance_rl/features.py)
// ============================================================

RLGuidanceNode::VelocityCmd RLGuidanceNode::decode_action(
    const std::array<float, 4>& a, float los_v, float los_z)
{
    float a0 = clamp(a[0], -1.0f, 1.0f);
    float a1 = clamp(a[1], -1.0f, 1.0f);
    float a2 = clamp(a[2], -1.0f, 1.0f);
    float a3 = clamp(a[3], -1.0f, 1.0f);

    float v_angle_v = clamp(los_v + a0 * dv_angle_max_, -elev_clamp_, elev_clamp_);
    float v_angle_z = los_z + a1 * dv_angle_max_;
    float speed     = speed_min_ + (speed_cmd_ - speed_min_) * (a2 + 1.0f) / 2.0f;
    float yaw_rate  = a3 * yaw_rate_max_;

    VelocityCmd cmd;
    angles_to_velocity(v_angle_v, v_angle_z, speed, cmd.vx, cmd.vy, cmd.vz);
    cmd.yaw_rate = yaw_rate;
    return cmd;
}

// ============================================================
//  PNG 回退控制器 (PNGTeacher — 1:1 对应 guidance_rl/png_teacher.py)
// ============================================================

RLGuidanceNode::PngResult RLGuidanceNode::png_step(
    const std::optional<std::tuple<int, int, int, int>>& det,
    float roll, float pitch, float yaw,
    float vx, float vy, float vz)
{
    PngResult r{};
    r.v_angle_v = png_d_angle_v_;
    r.v_angle_z = png_d_angle_z_;

    if (!det.has_value()) {
        // 目标丢失 — 不更新 PNG 角度，保持上次指令
        // 丢失处理由状态机 control_loop 接管
        r.vx = r.vy = r.vz = r.yaw_rate = 0;
        r.los_v = feat_los_v_;
        r.los_z = feat_los_z_;
        r.ex = feat_ex_;
        r.ey = feat_ey_;
        return r;
    }

    auto [x, y, w, h] = *det;
    if (w <= 0 || h <= 0) {
        r.vx = r.vy = r.vz = r.yaw_rate = 0;
        return r;
    }

    // 1) 像素误差
    float ex = (x + w / 2.0f) - image_width_ / 2.0f;
    float ey = (y + h / 2.0f) - image_height_ / 2.0f;
    r.ex = ex; r.ey = ey;

    // 2) LOS 角
    float los_v, los_z;
    pixel_to_los(ex, ey, focal_length_, roll, pitch, yaw, los_v, los_z);
    r.los_v = los_v; r.los_z = los_z;

    if (!png_init_) {
        // 首次检测 — 初始化
        png_d_angle_v_ = los_v;
        png_d_angle_z_ = los_z;
        png_last_los_v_ = los_v;
        png_last_los_z_ = los_z;
        png_last_v_angle_v_ = los_v;
        png_last_v_angle_z_ = los_z;
        png_last_ex_ = ex;
        png_d_yaw_ = 0;
        png_init_ = true;

        r.v_angle_v = los_v;
        r.v_angle_z = los_z;
        angles_to_velocity(los_v, los_z, speed_min_, r.vx, r.vy, r.vz);
        r.yaw_rate = 0;
        return r;
    }

    // 3) PNG 角度更新
    float diff_v = wrap_pi(los_v - png_last_los_v_);
    float diff_z = wrap_pi(los_z - png_last_los_z_);

    if (std::abs(diff_v) > los_diff_thresh_ || std::abs(diff_z) > los_diff_thresh_) {
        png_d_angle_v_ = kv_ * diff_v + png_last_v_angle_v_;
        png_d_angle_z_ = kz_ * diff_z + png_last_v_angle_z_;
        png_d_angle_v_ = clamp(png_d_angle_v_, -elev_clamp_, elev_clamp_);
    }

    // 4) Yaw PD
    float d_ex = ex - png_last_ex_;
    png_d_yaw_ = k1_yaw_ * ex + k2_yaw_ * d_ex;
    png_d_yaw_ = clamp(png_d_yaw_, -yaw_rate_max_, yaw_rate_max_);

    // 5) 速度合成
    float v_norm = std::sqrt(vx * vx + vy * vy + vz * vz);
    float d_v = clamp(v_norm + d_gain_, speed_min_, speed_cmd_);

    angles_to_velocity(png_d_angle_v_, png_d_angle_z_, d_v, r.vx, r.vy, r.vz);

    // 垂直视场补偿
    float vz_ey = clamp(k_ey_ * ey, -vz_ey_max_, vz_ey_max_);
    r.vz += vz_ey;

    r.yaw_rate = png_d_yaw_;
    r.v_angle_v = png_d_angle_v_;
    r.v_angle_z = png_d_angle_z_;

    // 更新状态
    png_last_los_v_ = los_v;
    png_last_los_z_ = los_z;
    png_last_v_angle_v_ = png_d_angle_v_;
    png_last_v_angle_z_ = png_d_angle_z_;
    png_last_ex_ = ex;

    return r;
}

void RLGuidanceNode::png_reset()
{
    png_init_ = false;
    png_d_angle_v_ = 0;
    png_d_angle_z_ = 0;
    png_last_los_v_ = 0;
    png_last_los_z_ = 0;
    png_last_v_angle_v_ = 0;
    png_last_v_angle_z_ = 0;
    png_last_ex_ = 0;
    png_d_yaw_ = 0;
}

// ============================================================
//  构造 & 析构
// ============================================================

RLGuidanceNode::RLGuidanceNode()
    : Node("uav_rl_guidance"),
      ort_env_(ORT_LOGGING_LEVEL_WARNING, "RLGuidance")
{
    // ---- 声明参数 ----
    this->declare_parameter("model_path",
        "/home/verser/ros2_ws/src/uav_rl_guidance/models/policy.onnx");
    this->declare_parameter("fallback_png", false);
    this->declare_parameter("focal_length", 1397.2f);
    this->declare_parameter("image_width", 1920);
    this->declare_parameter("image_height", 1080);
    this->declare_parameter("kv", 4.0f);
    this->declare_parameter("kz", 4.0f);
    this->declare_parameter("speed_cmd", 5.0f);
    this->declare_parameter("d_gain", 1.0f);
    this->declare_parameter("k1_yaw", 0.0005f);
    this->declare_parameter("k2_yaw", 0.0002f);
    this->declare_parameter("k_ey", 0.012f);
    this->declare_parameter("standby_altitude", -6.0f);
    this->declare_parameter("coast_thresh", 30);
    this->declare_parameter("lost_thresh", 90);
    this->declare_parameter("hit_radius", 0.8f);
    this->declare_parameter("csv_path",
        "/home/verser/ros2_ws/rl_intercept_stats.csv");
    this->declare_parameter("bench_test", false);
    this->declare_parameter("dv_angle_max", 1.2f);

    // ---- 读取参数 ----
    auto g_float  = [this](const char* n) { return this->get_parameter(n).as_double(); };
    auto g_int    = [this](const char* n) { return this->get_parameter(n).as_int(); };
    auto g_bool   = [this](const char* n) { return this->get_parameter(n).as_bool(); };
    auto g_str    = [this](const char* n) { return this->get_parameter(n).as_string(); };

    fallback_png_     = g_bool("fallback_png");
    bench_test_       = g_bool("bench_test");
    focal_length_     = static_cast<float>(g_float("focal_length"));
    image_width_      = g_int("image_width");
    image_height_     = g_int("image_height");
    kv_               = static_cast<float>(g_float("kv"));
    kz_               = static_cast<float>(g_float("kz"));
    speed_cmd_        = static_cast<float>(g_float("speed_cmd"));
    d_gain_           = static_cast<float>(g_float("d_gain"));
    k1_yaw_           = static_cast<float>(g_float("k1_yaw"));
    k2_yaw_           = static_cast<float>(g_float("k2_yaw"));
    k_ey_             = static_cast<float>(g_float("k_ey"));
    standby_altitude_ = static_cast<float>(g_float("standby_altitude"));
    coast_thresh_     = g_int("coast_thresh");
    lost_thresh_      = g_int("lost_thresh");
    hit_radius_       = static_cast<float>(g_float("hit_radius"));
    csv_path_         = g_str("csv_path");
    dv_angle_max_     = static_cast<float>(g_float("dv_angle_max"));
    std::string model_path = g_str("model_path");

    // ---- 加载 ONNX 模型 ----
    if (!fallback_png_) {
        try {
            Ort::SessionOptions opts;
            opts.SetIntraOpNumThreads(2);
            opts.SetGraphOptimizationLevel(
                GraphOptimizationLevel::ORT_ENABLE_ALL);

            ort_session_ = std::make_unique<Ort::Session>(
                ort_env_, model_path.c_str(), opts);

            auto name0 = ort_session_->GetInputNameAllocated(0, ort_allocator_);
            auto name1 = ort_session_->GetInputNameAllocated(1, ort_allocator_);
            auto oname0 = ort_session_->GetOutputNameAllocated(0, ort_allocator_);
            auto oname1 = ort_session_->GetOutputNameAllocated(1, ort_allocator_);

            ort_input_name_obs_    = name0.get();
            ort_input_name_h_      = name1.get();
            ort_output_name_action_ = oname0.get();
            ort_output_name_h_     = oname1.get();

            RCLCPP_INFO(this->get_logger(),
                "ONNX 模型已加载: %s", model_path.c_str());
            RCLCPP_INFO(this->get_logger(),
                "  输入: %s [1,15], %s [1,1,128]", name0.get(), name1.get());
            RCLCPP_INFO(this->get_logger(),
                "  输出: %s [1,4], %s [1,1,128]", oname0.get(), oname1.get());
        } catch (const Ort::Exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                "ONNX 模型加载失败: %s，回退到 PNG 模式", e.what());
            fallback_png_ = true;
        }
    } else {
        RCLCPP_WARN(this->get_logger(),
            "fallback_png=true，全程使用内置 PNG（基线模式）");
    }

    // ---- 初始化 ONNX 内存信息 ----
    ort_memory_info_ = Ort::MemoryInfo::CreateCpu(
        OrtArenaAllocator, OrtMemTypeDefault);

    // ---- 初始化 GRU 隐藏状态 ----
    gru_hidden_.fill(0);

    // ---- PX4 QoS ----
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(
            rmw_qos_profile_sensor_data.history, 5),
        rmw_qos_profile_sensor_data);

    // ---- 发布者 ----
    offboard_pub_ = this->create_publisher<OffboardControlMode>(
        "/px4_1/fmu/in/offboard_control_mode", 10);
    traj_pub_ = this->create_publisher<TrajectorySetpoint>(
        "/px4_1/fmu/in/trajectory_setpoint", 10);
    cmd_pub_ = this->create_publisher<VehicleCommand>(
        "/px4_1/fmu/in/vehicle_command", 10);
    data_pub_ = this->create_publisher<DataMsg>("/vpng_data", 10);

    // ---- 订阅者（导引用）----
    detect_sub_ = this->create_subscription<RectMsg>(
        "/camera_detect_result", 10,
        std::bind(&RLGuidanceNode::detect_cb, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<VehicleOdometry>(
        "/px4_1/fmu/out/vehicle_odometry", qos,
        std::bind(&RLGuidanceNode::odom_cb, this, std::placeholders::_1));
    local_pos_sub_ = this->create_subscription<VehicleLocalPosition>(
        "/px4_1/fmu/out/vehicle_local_position", qos,
        std::bind(&RLGuidanceNode::local_pos_cb, this, std::placeholders::_1));
    status_sub_ = this->create_subscription<VehicleStatus>(
        "/px4_1/fmu/out/vehicle_status", qos,
        std::bind(&RLGuidanceNode::status_cb, this, std::placeholders::_1));
    hover_sub_ = this->create_subscription<HoverThrustEstimate>(
        "/px4_1/fmu/out/hover_thrust_estimate", qos,
        std::bind(&RLGuidanceNode::hover_cb, this, std::placeholders::_1));

    // ---- 订阅者（纯统计用）----
    self_gps_sub_ = this->create_subscription<SensorGps>(
        "/px4_1/fmu/out/vehicle_gps_position", qos,
        std::bind(&RLGuidanceNode::self_gps_cb, this, std::placeholders::_1));
    target_gps_sub_ = this->create_subscription<SensorGps>(
        "/px4_2/fmu/out/vehicle_gps_position", qos,
        std::bind(&RLGuidanceNode::target_gps_cb, this, std::placeholders::_1));

    // ---- 定时器 ----
    timer_control_ = this->create_wall_timer(
        std::chrono::milliseconds(20),  // 50 Hz
        std::bind(&RLGuidanceNode::control_loop, this));
    timer_policy_ = this->create_wall_timer(
        std::chrono::milliseconds(50),  // 20 Hz
        std::bind(&RLGuidanceNode::policy_tick, this));

    // ---- Bench test 模式 ----
    if (bench_test_) {
        state_ = RLState::SEARCHING;
        RCLCPP_WARN(this->get_logger(),
            "bench_test=true：跳过起飞，直接 SEARCHING（仅台架调试）");
    }

    RCLCPP_INFO(this->get_logger(),
        "RLGuidance 节点已启动 | fallback_png=%d standby=%.1f hit_r=%.1f",
        fallback_png_, standby_altitude_, hit_radius_);
}

RLGuidanceNode::~RLGuidanceNode()
{
    // 写入 CSV 摘要
    if (csv_file_.is_open()) {
        csv_file_ << "\n# ========== 统计摘要 ==========\n";
        csv_file_ << "# 总检测帧数," << total_detect_frames_ << "\n";
        csv_file_ << "# 累计丢失帧数," << lost_frames_total_ << "\n";
        csv_file_ << "# 最近接距离(m)," << min_distance_ << "\n";
        csv_file_ << "# watchdog触发次数," << watchdog_total_ << "\n";
        csv_file_.close();
        RCLCPP_INFO(this->get_logger(), "[统计] CSV 已保存至: %s",
                    csv_path_.c_str());
    }
}

// ============================================================
//  50 Hz 主控制循环
// ============================================================

void RLGuidanceNode::control_loop()
{
    switch (state_) {
    case RLState::TAKE_OFF:    handle_takeoff(); break;
    case RLState::SEARCHING:   handle_searching(); break;
    case RLState::INTERCEPT:   handle_intercept_heartbeat(); break;
    case RLState::TRACK_LOST:  handle_track_lost(); break;
    case RLState::DONE:        handle_done(); break;
    }
}

void RLGuidanceNode::handle_takeoff()
{
    arm();
    set_offboard_mode();
    publish_offboard_position_mode();
    publish_position_setpoint(0, 0, standby_altitude_);

    if (takeoff_counter_ < takeoff_frames_) {
        takeoff_counter_++;
        return;
    }

    float alt_err = std::abs(local_z_ - standby_altitude_);
    if (hover_thrust_ok_ && alt_err < 0.5f) {
        RCLCPP_INFO(this->get_logger(),
            "→ 起飞完成 z=%.2f，进入 SEARCHING", local_z_);
        state_ = RLState::SEARCHING;
    }
}

void RLGuidanceNode::handle_searching()
{
    publish_offboard_velocity_mode();
    publish_velocity_setpoint(0, 0, 0, 0);
    // 检测回调切换到 INTERCEPT
}

void RLGuidanceNode::handle_intercept_heartbeat()
{
    // 50 Hz: 心跳 + 丢失分级; 速度指令由 20 Hz policy_tick 更新 coast 缓存
    arm();
    set_offboard_mode();

    if (lost_frames_ >= lost_thresh_) {
        RCLCPP_WARN(this->get_logger(),
            "[INTERCEPT] 目标长时间丢失 lost=%d，切换 TRACK_LOST", lost_frames_);
        state_ = RLState::TRACK_LOST;
        return;
    }

    publish_offboard_velocity_mode();
    publish_velocity_setpoint(coast_vx_, coast_vy_, coast_vz_, coast_yaw_);
}

void RLGuidanceNode::handle_track_lost()
{
    // 制动 + 慢旋搜索
    float yaw_rate = (std::abs(coast_yaw_) > 0.01f) ? coast_yaw_ : search_yaw_rate_;
    publish_offboard_velocity_mode();
    publish_velocity_setpoint(0, 0, 0, yaw_rate);
    // 检测回调恢复 INTERCEPT
}

void RLGuidanceNode::handle_done()
{
    publish_offboard_velocity_mode();
    publish_velocity_setpoint(0, 0, 0, 0);
}

// ============================================================
//  20 Hz 策略推理
// ============================================================

void RLGuidanceNode::policy_tick()
{
    if (state_ != RLState::INTERCEPT) return;

    auto det = detect_;
    if (!det.has_value() && lost_frames_ < coast_thresh_) {
        return;  // 惯性续飞，保持 coast 缓存
    }

    if (fallback_png_) {
        // 纯 PNG 模式
        auto r = png_step(det, roll_, pitch_, yaw_, vx_, vy_, vz_);
        coast_vx_ = r.vx; coast_vy_ = r.vy; coast_vz_ = r.vz; coast_yaw_ = r.yaw_rate;
        last_cmd_source_ = "png";
        publish_debug_data(r.los_v, r.los_z, r.ex, r.ey);
    } else if (watchdog_count_ > 0) {
        // Watchdog latch — 连续 N 帧回退 PNG
        watchdog_count_--;
        auto r = png_step(det, roll_, pitch_, yaw_, vx_, vy_, vz_);
        coast_vx_ = r.vx; coast_vy_ = r.vy; coast_vz_ = r.vz; coast_yaw_ = r.yaw_rate;
        last_cmd_source_ = "png_watchdog";
        publish_debug_data(r.los_v, r.los_z, r.ex, r.ey);
    } else {
        // 策略推理
        try {
            auto obs = build_features(det, roll_, pitch_, yaw_,
                                      vx_, vy_, vz_, local_z_, 0.05f);
            auto [action, h_new] = policy_inference(obs, gru_hidden_);

            // Watchdog 检查: NaN/Inf
            for (float v : action) {
                if (!std::isfinite(v)) {
                    throw std::runtime_error("策略输出含 NaN/Inf");
                }
            }

            auto cmd = decode_action(action, feat_los_v_, feat_los_z_);
            float speed = std::sqrt(cmd.vx * cmd.vx + cmd.vy * cmd.vy + cmd.vz * cmd.vz);
            if (!std::isfinite(speed) || speed > 1.5f * speed_cmd_) {
                throw std::runtime_error(
                    "速度指令越界: |v|=" + std::to_string(speed));
            }

            coast_vx_ = cmd.vx; coast_vy_ = cmd.vy;
            coast_vz_ = cmd.vz; coast_yaw_ = cmd.yaw_rate;
            gru_hidden_ = h_new;
            last_cmd_source_ = "policy";
            publish_debug_data(feat_los_v_, feat_los_z_, feat_ex_, feat_ey_);

        } catch (const std::exception& e) {
            watchdog_count_ = WATCHDOG_LATCH;
            watchdog_total_++;
            gru_hidden_.fill(0);  // 重置隐藏状态避免污染传播
            RCLCPP_WARN(this->get_logger(),
                "[watchdog] 策略异常(%s)，回退 PNG %d 帧 (累计 %d 次)",
                e.what(), WATCHDOG_LATCH, watchdog_total_);

            auto r = png_step(det, roll_, pitch_, yaw_, vx_, vy_, vz_);
            coast_vx_ = r.vx; coast_vy_ = r.vy; coast_vz_ = r.vz; coast_yaw_ = r.yaw_rate;
            last_cmd_source_ = "png_watchdog";
            publish_debug_data(r.los_v, r.los_z, r.ex, r.ey);
        }
    }

    save_stats_to_csv();
}

// ============================================================
//  回调
// ============================================================

void RLGuidanceNode::detect_cb(const RectMsg::SharedPtr msg)
{
    if (msg->width == -1) {
        detect_ = std::nullopt;
        lost_frames_++;
        lost_frames_total_++;
    } else {
        detect_ = std::make_tuple(
            static_cast<int>(msg->x), static_cast<int>(msg->y),
            static_cast<int>(msg->width), static_cast<int>(msg->height));
        lost_frames_ = 0;

        if (state_ == RLState::SEARCHING) {
            RCLCPP_INFO(this->get_logger(), "→ 视觉检测到目标，进入 INTERCEPT");
            // 重置策略 & PNG 状态
            gru_hidden_.fill(0);
            has_los_ = false;
            feat_age_ = 0;
            png_reset();
            state_ = RLState::INTERCEPT;
        } else if (state_ == RLState::TRACK_LOST) {
            RCLCPP_INFO(this->get_logger(), "→ 目标重新出现，恢复 INTERCEPT");
            gru_hidden_.fill(0);
            has_los_ = false;
            feat_age_ = 0;
            png_reset();
            state_ = RLState::INTERCEPT;
        }
    }
    total_detect_frames_++;
}

void RLGuidanceNode::odom_cb(const VehicleOdometry::SharedPtr msg)
{
    quat_to_euler(msg->q[0], msg->q[1], msg->q[2], msg->q[3],
                  roll_, pitch_, yaw_);
}

void RLGuidanceNode::local_pos_cb(const VehicleLocalPosition::SharedPtr msg)
{
    vx_ = msg->vx;
    vy_ = msg->vy;
    vz_ = msg->vz;
    local_z_ = msg->z;
}

void RLGuidanceNode::status_cb(const VehicleStatus::SharedPtr msg)
{
    offboard_active_ =
        (msg->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD);
}

void RLGuidanceNode::hover_cb(const HoverThrustEstimate::SharedPtr msg)
{
    if (msg->hover_thrust_var < 0.0025f) {
        hover_thrust_ok_ = true;
    }
}

void RLGuidanceNode::gps_to_ned(double lat, double lon, double alt,
                                double& n, double& e, double& d)
{
    n = (lat - gps_origin_lat_) * kMPerDegLat;
    e = (lon - gps_origin_lon_) * kMPerDegLat *
        std::cos(gps_origin_lat_ * M_PI / 180.0);
    d = -(alt - gps_origin_alt_);
}

void RLGuidanceNode::self_gps_cb(const SensorGps::SharedPtr msg)
{
    if (!gps_origin_set_) {
        gps_origin_lat_ = msg->latitude_deg;
        gps_origin_lon_ = msg->longitude_deg;
        gps_origin_alt_ = msg->altitude_msl_m;
        gps_origin_set_ = true;
        RCLCPP_INFO(this->get_logger(),
            "[统计] 坐标系原点初始化: lat=%.7f lon=%.7f",
            msg->latitude_deg, msg->longitude_deg);
    }
    double n, e, d;
    gps_to_ned(msg->latitude_deg, msg->longitude_deg, msg->altitude_msl_m,
               n, e, d);
    self_pos_stats_ = Eigen::Vector3f(
        static_cast<float>(n), static_cast<float>(e), static_cast<float>(d));
}

void RLGuidanceNode::target_gps_cb(const SensorGps::SharedPtr msg)
{
    if (!gps_origin_set_) return;
    double n, e, d;
    gps_to_ned(msg->latitude_deg, msg->longitude_deg, msg->altitude_msl_m,
               n, e, d);
    target_pos_stats_ = Eigen::Vector3f(
        static_cast<float>(n), static_cast<float>(e), static_cast<float>(d));
    target_pos_ok_ = true;
}

// ============================================================
//  PX4 发布辅助
// ============================================================

void RLGuidanceNode::arm()
{
    publish_vehicle_command(
        VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

void RLGuidanceNode::set_offboard_mode()
{
    publish_vehicle_command(
        VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
}

void RLGuidanceNode::publish_offboard_position_mode()
{
    OffboardControlMode msg{};
    msg.position  = true;
    msg.velocity  = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_pub_->publish(msg);
}

void RLGuidanceNode::publish_offboard_velocity_mode()
{
    OffboardControlMode msg{};
    msg.position  = false;
    msg.velocity  = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_pub_->publish(msg);
}

void RLGuidanceNode::publish_position_setpoint(float x, float y, float z)
{
    TrajectorySetpoint msg{};
    msg.position  = {x, y, z};
    msg.yaw       = yaw_;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    traj_pub_->publish(msg);
}

void RLGuidanceNode::publish_velocity_setpoint(
    float vx, float vy, float vz, float yaw_rate)
{
    TrajectorySetpoint msg{};
    msg.position  = {std::nanf(""), std::nanf(""), std::nanf("")};
    msg.velocity  = {vx, vy, vz};
    msg.yaw       = std::nanf("");
    msg.yawspeed  = yaw_rate;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    traj_pub_->publish(msg);
}

void RLGuidanceNode::publish_vehicle_command(uint16_t command, float p1, float p2)
{
    VehicleCommand msg{};
    msg.param1           = p1;
    msg.param2           = p2;
    msg.command          = command;
    msg.target_system    = 2;    // px4_1 (Instance 1 → DDS key 2)
    msg.target_component = 1;
    msg.source_system    = 1;
    msg.source_component = 1;
    msg.from_external    = true;
    msg.timestamp        = this->get_clock()->now().nanoseconds() / 1000;
    cmd_pub_->publish(msg);
}

void RLGuidanceNode::publish_debug_data(float los_v, float los_z,
                                        float ex, float ey)
{
    DataMsg msg{};
    msg.los_angle_v = los_v;
    msg.los_angle_z = los_z;
    msg.ex = ex;
    msg.ey = ey;
    msg.yaw   = yaw_;
    msg.pitch = pitch_;
    msg.roll  = roll_;
    msg.vel_total = std::sqrt(vx_ * vx_ + vy_ * vy_ + vz_ * vz_);
    data_pub_->publish(msg);
}

// ============================================================
//  CSV 统计
// ============================================================

void RLGuidanceNode::save_stats_to_csv(bool just_hit)
{
    (void)just_hit;

    if (!csv_file_.is_open()) {
        csv_file_.open(csv_path_);
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "[统计] 无法打开 CSV: %s",
                         csv_path_.c_str());
            return;
        }
        csv_file_ << "time_s,self_x,self_y,self_z,target_x,target_y,target_z,"
                     "dist_m,los_angle_v,los_angle_z,d_v_angle_v,d_v_angle_z,"
                     "ex,ey,vx,vy,vz,detect_w,state\n";
        RCLCPP_INFO(this->get_logger(), "[统计] CSV 已创建: %s", csv_path_.c_str());
    }

    double t = this->get_clock()->now().nanoseconds() * 1e-9;

    // 距离计算（纯统计用）
    double dist = -1.0;
    bool just_hit_local = false;
    if (target_pos_ok_ && gps_origin_set_) {
        dist = (self_pos_stats_ - target_pos_stats_).norm();
        if (dist < min_distance_) min_distance_ = dist;
        if (dist < hit_radius_ && !hit_recorded_) {
            hit_recorded_ = true;
            just_hit_local = true;
            RCLCPP_WARN(this->get_logger(),
                "★ [统计] 检测到命中！真实距离=%.3f m (统计专用)", dist);
            state_ = RLState::DONE;
        }
    }

    // 节流：每 2 次写一行
    csv_counter_++;
    if (csv_counter_ % 2 != 0 && !just_hit_local) return;

    float sp_x = self_pos_stats_(0), sp_y = self_pos_stats_(1), sp_z = self_pos_stats_(2);
    float tp_x = target_pos_stats_(0), tp_y = target_pos_stats_(1), tp_z = target_pos_stats_(2);
    int det_w = detect_.has_value() ? std::get<2>(*detect_) : -1;

    float los_v = feat_los_v_;
    float los_z = feat_los_z_;
    float ex = feat_ex_;
    float ey = feat_ey_;

    csv_file_ << std::fixed << std::setprecision(4)
              << t << ","
              << sp_x << "," << sp_y << "," << -sp_z << ","
              << tp_x << "," << tp_y << "," << -tp_z << ","
              << dist << ","
              << los_v << "," << los_z << ",0,0,"
              << ex << "," << ey << ","
              << vx_ << "," << vy_ << "," << vz_ << ","
              << det_w << "," << static_cast<int>(state_) << "\n";
    csv_file_.flush();
}

// ============================================================
//  main
// ============================================================

int main(int argc, char* argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RLGuidanceNode>());
    rclcpp::shutdown();
    return 0;
}

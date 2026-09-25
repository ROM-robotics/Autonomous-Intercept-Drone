/**
 * vision_png_control.cpp
 *
 * 基于视觉的比例导引（Vision-based PNG）实现
 *
 * ┌──────────────────────────────────────────────────────────────┐
 * │  导引算法（纯视觉，严禁使用目标 GPS）                          │
 * │    1. 从像素坐标 → 相机系 LOS 向量                           │
 * │    2. R_c2b * R_b2n 旋转到 NED 系                           │
 * │    3. PNG：d_v_angle = N * dLOS + last_v_angle              │
 * │    4. 速度指令：vx/vy/vz                                      │
 * ├──────────────────────────────────────────────────────────────┤
 * │  统计记录（仅用于数据分析，不参与导引计算）                     │
 * │    订阅 px4_2 GPS → LocalCartesian → 计算真实距离 → CSV       │
 * └──────────────────────────────────────────────────────────────┘
 *
 * 参考：
 *   uav_ibvs_control/src/los_control.cpp    —— 视觉 LOS 计算方式
 *   uav_png_intercept/src/png_control.cpp   —— PNG 状态机框架
 *   include/uav_vision_png/vision_png_control.hpp
 */

#include "uav_vision_png/vision_png_control.hpp"
#include <iomanip>
#include <cmath>

// ============================================================
//  构造函数
// ============================================================
VisionPNG::VisionPNG() : Node("uav_vision_png")
{
    // ---- QoS（与 PX4 MiddleWare 一致，sensor_data 模式）----
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // ---- 从参数服务器读取参数（允许 launch 文件覆盖）----
    this->declare_parameter("focal_length",      focal_length_);
    this->declare_parameter("image_width",        image_width_);
    this->declare_parameter("image_height",       image_height_);
    this->declare_parameter("N_png",              N_png_);
    this->declare_parameter("speed_cmd",          speed_cmd_);
    this->declare_parameter("d_gain",             d_gain_);
    this->declare_parameter("kv",                 kv_);
    this->declare_parameter("kz",                 kz_);
    this->declare_parameter("k1_yaw",             k1_yaw_);
    this->declare_parameter("k2_yaw",             k2_yaw_);
    this->declare_parameter("k_ey",               k_ey_);
    this->declare_parameter("standby_altitude",   standby_altitude_);
    this->declare_parameter("coast_thresh",       coast_thresh_);
    this->declare_parameter("lost_thresh",        lost_thresh_);
    this->declare_parameter("hit_radius",         (double)hit_radius_);
    this->declare_parameter("csv_path",           csv_path_);

    focal_length_      = this->get_parameter("focal_length").as_double();
    image_width_       = this->get_parameter("image_width").as_int();
    image_height_      = this->get_parameter("image_height").as_int();
    N_png_             = this->get_parameter("N_png").as_double();
    speed_cmd_         = this->get_parameter("speed_cmd").as_double();
    d_gain_            = this->get_parameter("d_gain").as_double();
    kv_                = this->get_parameter("kv").as_double();
    kz_                = this->get_parameter("kz").as_double();
    k1_yaw_            = this->get_parameter("k1_yaw").as_double();
    k2_yaw_            = this->get_parameter("k2_yaw").as_double();
    k_ey_              = this->get_parameter("k_ey").as_double();
    standby_altitude_  = this->get_parameter("standby_altitude").as_double();
    coast_thresh_      = this->get_parameter("coast_thresh").as_int();
    lost_thresh_       = this->get_parameter("lost_thresh").as_int();
    hit_radius_        = (float)this->get_parameter("hit_radius").as_double();
    csv_path_          = this->get_parameter("csv_path").as_string();

    // ---- 相机→机体旋转矩阵（与 los_control.cpp 保持一致）----
    // Cx → By,  Cy → Bz,  Cz → Bx
    R_c2b_ << 0, 0, 1,
               1, 0, 0,
               0, 1, 0;

    // ============================================================
    //  发布者
    // ============================================================
    offboard_pub_ = create_publisher<OffboardControlMode>(
        "/px4_1/fmu/in/offboard_control_mode", 10);
    traj_pub_ = create_publisher<TrajectorySetpoint>(
        "/px4_1/fmu/in/trajectory_setpoint", 10);
    cmd_pub_ = create_publisher<VehicleCommand>(
        "/px4_1/fmu/in/vehicle_command", 10);
    data_pub_ = create_publisher<DataMsg>("/vpng_data", 10);

    // ============================================================
    //  订阅者 —— 导引专用
    // ============================================================

    // 视觉检测结果（核心输入）
    detect_sub_ = create_subscription<RectMsg>(
        "/camera_detect_result", 10,
        [this](const RectMsg::UniquePtr msg) {
            detect_x_ = msg->x;
            detect_y_ = msg->y;
            detect_w_ = msg->width;
            detect_h_ = msg->height;
            detect_fresh_ = true;

            if (msg->width == -1) {
                lost_frames_++;
            } else {
                lost_frames_ = 0;
                // 检测有效：若正在 SEARCHING，切换到拦截
                if (state_ == VpngState::SEARCHING) {
                    RCLCPP_INFO(get_logger(), "→ 视觉检测到目标，进入 INTERCEPT");
                    state_ = VpngState::INTERCEPT;
                }
                // 若目标重新出现（从 TRACK_LOST 恢复）
                if (state_ == VpngState::TRACK_LOST) {
                    RCLCPP_INFO(get_logger(), "→ 目标重新出现，恢复 INTERCEPT");
                    png_initialized_ = false;  // 重新初始化 PNG
                    state_ = VpngState::INTERCEPT;
                }
            }

            // 记录检测时间戳（ms）
            auto now = std::chrono::system_clock::now();
            detect_timestamp_ = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()).count();
        });

    // 自身姿态（四元数 → 欧拉角）
    odometry_sub_ = create_subscription<VehicleOdometry>(
        "/px4_1/fmu/out/vehicle_odometry", qos,
        [this](const VehicleOdometry::SharedPtr msg) {
            double q0 = msg->q[0], q1 = msg->q[1];
            double q2 = msg->q[2], q3 = msg->q[3];
            roll_  = std::atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
            pitch_ = std::asin (2*(q0*q2 - q3*q1));
            yaw_   = std::atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
        });

    // 自身速度与本地高度
    local_pos_sub_ = create_subscription<VehicleLocalPosition>(
        "/px4_1/fmu/out/vehicle_local_position", qos,
        [this](const VehicleLocalPosition::SharedPtr msg) {
            vx_ = msg->vx;
            vy_ = msg->vy;
            vz_ = msg->vz;
            local_z_ = msg->z;
        });

    // 飞行状态（判断 Offboard）
    status_sub_ = create_subscription<VehicleStatus>(
        "/px4_1/fmu/out/vehicle_status", qos,
        [this](const VehicleStatus::SharedPtr msg) {
            offboard_active_ =
                (msg->nav_state == msg->NAVIGATION_STATE_OFFBOARD);
        });

    // 悬停推力估计（判断起飞完成）
    hover_sub_ = create_subscription<HoverThrustEstimate>(
        "/px4_1/fmu/out/hover_thrust_estimate", qos,
        [this](const HoverThrustEstimate::SharedPtr msg) {
            if (msg->hover_thrust_var < 0.0025f) {
                hover_thrust_    = msg->hover_thrust;
                hover_thrust_ok_ = true;
            }
        });

    // ============================================================
    //  订阅者 —— 纯统计用（导引算法严禁使用这些数据）
    // ============================================================

    // 自身 GPS → 建立统计坐标系
    self_gps_sub_ = create_subscription<SensorGps>(
        "/px4_1/fmu/out/vehicle_gps_position", qos,
        [this](const SensorGps::SharedPtr msg) {
            if (!lc_stats_init_) {
                // 以首帧 GPS 作为 LocalCartesian 原点（仅统计用）
                lc_stats_ = GeographicLib::LocalCartesian(
                    msg->latitude_deg,
                    msg->longitude_deg,
                    msg->altitude_msl_m);
                lc_stats_init_ = true;
                RCLCPP_INFO(get_logger(),
                    "[统计] 坐标系原点初始化: lat=%.7f lon=%.7f",
                    msg->latitude_deg, msg->longitude_deg);
            }
            // 更新自身位置（ENU → NED）
            double e, n, u;
            lc_stats_.Forward(msg->latitude_deg, msg->longitude_deg,
                              msg->altitude_msl_m, e, n, u);
            self_pos_stats_ = {(float)n, (float)e, (float)(-u)};
        });

    // 目标 GPS（px4_2，仅供统计！导引算法不得使用）
    target_gps_sub_ = create_subscription<SensorGps>(
        "/px4_2/fmu/out/vehicle_gps_position", qos,
        [this](const SensorGps::SharedPtr msg) {
            if (!lc_stats_init_) return;
            double e, n, u;
            lc_stats_.Forward(msg->latitude_deg, msg->longitude_deg,
                              msg->altitude_msl_m, e, n, u);
            target_pos_stats_  = {(float)n, (float)e, (float)(-u)};
            target_pos_ok_    = true;
        });

    // ============================================================
    //  定时器
    // ============================================================
    // 200 Hz 主控制循环（心跳 + 状态机调度）
    timer_control_ = create_wall_timer(
        std::chrono::microseconds(5000),
        std::bind(&VisionPNG::control_loop, this));

    // 20 Hz 视觉 PNG LOS 计算
    timer_png_ = create_wall_timer(
        std::chrono::milliseconds(25),
        std::bind(&VisionPNG::png_calculate, this));

    RCLCPP_INFO(get_logger(),
        "VisionPNG 节点已启动 | focal=%.1f img=%dx%d N=%.1f spd=%.1f",
        focal_length_, image_width_, image_height_, N_png_, speed_cmd_);
}

// ============================================================
//  析构函数 —— 关闭 CSV 文件
// ============================================================
VisionPNG::~VisionPNG()
{
    if (csv_file_.is_open()) {
        // 写入摘要行
        csv_file_ << "\n# ========== 统计摘要 ==========\n";
        csv_file_ << "# 总检测帧数,"    << stats_.total_detect_frames << "\n";
        csv_file_ << "# 累计丢失帧数,"  << stats_.lost_frames_total   << "\n";
        csv_file_ << "# 最近接距离(m)," << stats_.min_distance         << "\n";
        if (stats_.hit_recorded) {
            csv_file_ << "# 命中时刻(s)," << stats_.intercept_time << "\n";
        }
        csv_file_.close();
        RCLCPP_INFO(get_logger(), "[统计] CSV 已保存至: %s", csv_path_.c_str());
    }
    std::cout << "VisionPNG 节点已退出" << std::endl;
}

// ============================================================
//  200 Hz 主控制循环
// ============================================================
void VisionPNG::control_loop()
{
    auto now = std::chrono::system_clock::now();
    control_timestamp_ = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
    time_diff_ = control_timestamp_ - detect_timestamp_;

    switch (state_) {
    case VpngState::TAKE_OFF:    handle_takeoff();    break;
    case VpngState::SEARCHING:   handle_searching();  break;
    case VpngState::INTERCEPT:   handle_intercept();  break;
    case VpngState::TRACK_LOST:  handle_track_lost(); break;
    case VpngState::DONE:        handle_done();       break;
    }
}

// ============================================================
//  20 Hz  视觉 PNG LOS 计算
//  严格参考 los_control.cpp 中的 LOS_calculate()
//  输入：像素坐标 detect_x_, detect_y_, detect_w_, detect_h_
//  输出：d_v_angle_v_, d_v_angle_z_（期望速度角度）
// ============================================================
void VisionPNG::png_calculate()
{
    // 只有在拦截阶段且目标有效时才更新
    if (state_ != VpngState::INTERCEPT) return;
    if (detect_w_ == -1) return;  // 目标丢失，不更新

    // ---------- Step 1: 计算像素误差（相对图像中心）----------
    double cx = image_width_  / 2.0;
    double cy = image_height_ / 2.0;
    // bbox 中心
    ex_ = (detect_x_ + detect_w_ / 2.0) - cx;
    ey_ = (detect_y_ + detect_h_ / 2.0) - cy;

    // ---------- Step 2: 构造相机系 LOS 向量 ----------
    // 与 los_control.cpp 一致：Nt = [ex, ey, focal]
    Eigen::Vector3d Nt(ex_, ey_, focal_length_);

    // ---------- Step 3: 相机系 → 机体系 → NED 系 ----------
    // 机体→NED 旋转矩阵（ZYX 欧拉角，与 los_control.cpp 完全一致）
    Eigen::AngleAxisd Ryaw  (yaw_,   Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Rpitch(pitch_, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rroll (roll_,  Eigen::Vector3d::UnitX());
    Eigen::Matrix3d R_b2n = Ryaw.toRotationMatrix()
                          * Rpitch.toRotationMatrix()
                          * Rroll.toRotationMatrix();

    // LOS 向量：相机系 → 机体系 → NED 系
    Eigen::Vector3d Nt_ned = R_b2n * R_c2b_ * Nt;

    // 检测是否与上一帧完全相同（重复帧，跳过更新）
    if (Nt_ned == last_LOS_vec_) return;
    last_LOS_vec_ = Nt_ned;
    LOS_vec_      = Nt_ned;

    // ---------- Step 4: 计算 LOS 角度 ----------
    double rxy = std::sqrt(Nt_ned(0)*Nt_ned(0) + Nt_ned(1)*Nt_ned(1));
    LOS_angle_v_ = std::atan2(Nt_ned(2), rxy);       // 仰角（NED Down 为正）
    LOS_angle_z_ = std::atan2(Nt_ned(0), Nt_ned(1)); // 方位角（北=0，顺时针）

    // ---------- Step 5: PNG 角度更新 ----------
    // 仅首次进入时初始化，用 LOS 方向作为初始速度方向
    // 注意：不再用 |V|<1 重置，避免加速阶段反复重置导致振荡
    Eigen::Vector3d Vt(vx_, vy_, vz_);
    if (!png_initialized_) {
        d_v_angle_v_      = LOS_angle_v_;
        d_v_angle_z_      = LOS_angle_z_;
        last_LOS_angle_v_ = LOS_angle_v_;
        last_LOS_angle_z_ = LOS_angle_z_;
        last_v_angle_v_   = LOS_angle_v_;  // 用 LOS 而非实测速度，避免起飞残余污染
        last_v_angle_z_   = LOS_angle_z_;
        png_initialized_  = true;
        RCLCPP_INFO(get_logger(),
            "[PNG 初始化] LOS(v=%.3f z=%.3f) |V|=%.2f",
            LOS_angle_v_, LOS_angle_z_, Vt.norm());
        return;
    }

    double diff_LOS_v = LOS_angle_v_ - last_LOS_angle_v_;
    double diff_LOS_z = LOS_angle_z_ - last_LOS_angle_z_;

    // 处理 ±π 穿越（方位角）
    while (diff_LOS_z >  M_PI) diff_LOS_z -= 2.0 * M_PI;
    while (diff_LOS_z < -M_PI) diff_LOS_z += 2.0 * M_PI;
    while (diff_LOS_v >  M_PI) diff_LOS_v -= 2.0 * M_PI;
    while (diff_LOS_v < -M_PI) diff_LOS_v += 2.0 * M_PI;

    // LOS 变化量足够大才更新（与 los_control.cpp 阈值 0.02 一致）
    if (std::abs(diff_LOS_v) > 0.02 || std::abs(diff_LOS_z) > 0.02) {
        // 用「上一帧的期望速度角」作为 PNG 基准（固定参考，不跟随实测速度）
        // 这样可以避免大机动时实测速度滞后导致的 PNG 发散
        // PNG 核心公式（与 los_control.cpp 完全一致）
        d_v_angle_v_ = kv_ * diff_LOS_v + last_v_angle_v_;
        d_v_angle_z_ = kz_ * diff_LOS_z + last_v_angle_z_;

        // 限制仰角范围（±45°），防止仰角过大导致无人机猛冲高空
        d_v_angle_v_ = std::max(-M_PI/4.0, std::min(M_PI/4.0, d_v_angle_v_));

        // 更新 last_v_angle 为本次期望值（迭代更新）
        last_v_angle_v_ = d_v_angle_v_;
        last_v_angle_z_ = d_v_angle_z_;

        last_LOS_angle_v_ = LOS_angle_v_;
        last_LOS_angle_z_ = LOS_angle_z_;

        RCLCPP_INFO(get_logger(),
            "[PNG] ex=%.1f ey=%.1f | LOS(v=%.3f z=%.3f) | dLOS(v=%.3f z=%.3f) | dV(v=%.3f z=%.3f)",
            ex_, ey_,
            LOS_angle_v_, LOS_angle_z_,
            diff_LOS_v, diff_LOS_z,
            d_v_angle_v_, d_v_angle_z_);
    }

    // ---------- Step 6: 主动偏航跟踪（FOV Lock-On）----------
    // 将水平像素误差 ex 直接转换为偏航角速率指令，强制机头对准目标
    // d_yaw = k1 * ex + k2 * d(ex)/dt   [PD 控制，单位 rad/s]
    // ex > 0 表示目标在图像右侧 → 需要正偏航（顺时针 in NED）
    {
        double d_ex = ex_ - last_ex_;
        d_yaw_   = k1_yaw_ * ex_ + k2_yaw_ * d_ex;
        // 限制偏航角速率到 ±1.0 rad/s（约 57°/s），避免过激旋转
        d_yaw_ = std::max(-1.0, std::min(1.0, d_yaw_));
        last_ex_ = ex_;
    }

    // ---------- Step 7: 发布 /vpng_data 调试话题 ----------
    DataMsg data{};
    data.d_v_angle_v   = (float)d_v_angle_v_;
    data.d_v_angle_z   = (float)d_v_angle_z_;
    data.v_angle_v     = (float)std::atan2(Vt(2), std::sqrt(Vt(1)*Vt(1)+Vt(0)*Vt(0)));
    data.v_angle_z     = (float)std::atan2(Vt(0), Vt(1));
    data.los_angle_v   = (float)LOS_angle_v_;
    data.los_angle_z   = (float)LOS_angle_z_;
    data.diff_los_angle_v = (float)(LOS_angle_v_ - last_LOS_angle_v_);
    data.diff_los_angle_z = (float)(LOS_angle_z_ - last_LOS_angle_z_);
    data.ex            = (float)ex_;
    data.ey            = (float)ey_;
    data.yaw           = (float)yaw_;
    data.pitch         = (float)pitch_;
    data.roll          = (float)roll_;
    data.pixel_u       = (float)(detect_x_ + detect_w_ / 2.0);
    data.pixel_v       = (float)(detect_y_ + detect_h_ / 2.0);
    data.vel_total     = (float)Vt.norm();
    data_pub_->publish(data);

    // ---------- Step 8: 更新统计计数 ----------
    stats_.total_detect_frames++;
    if (detect_w_ == -1) stats_.lost_frames_total++;
}

// ============================================================
//  状态机：TAKE_OFF
//  位置控制爬升到 standby_altitude_，等待悬停推力收敛
// ============================================================
void VisionPNG::handle_takeoff()
{
    // 持续发送 arm + offboard 心跳
    arm();
    set_offboard_mode();
    publish_offboard_position_mode();
    publish_position_setpoint(0.0f, 0.0f, (float)standby_altitude_);

    if (takeoff_counter_ < takeoff_frames_) {
        takeoff_counter_++;
        return;
    }

    // 高度到位且悬停推力已收敛
    float alt_err = std::abs(local_z_ - (float)standby_altitude_);
    if (hover_thrust_ok_ && alt_err < 0.5f) {
        RCLCPP_INFO(get_logger(),
            "→ 起飞完成 z=%.2f hover_thrust=%.3f，进入 SEARCHING",
            local_z_, hover_thrust_);
        state_ = VpngState::SEARCHING;
    } else {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "[TAKEOFF] z=%.2f/%.2f hover_ok=%s",
            local_z_, (float)standby_altitude_,
            hover_thrust_ok_ ? "YES" : "NO");
    }
}

// ============================================================
//  状态机：SEARCHING
//  已到达待机高度，等待视觉检测到目标
// ============================================================
void VisionPNG::handle_searching()
{
    // 悬停等待目标出现
    publish_offboard_velocity_mode();
    publish_velocity_setpoint(0.0f, 0.0f, 0.0f, 0.0f);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "[SEARCHING] 等待视觉检测目标... detect_w=%d", detect_w_);

    // 目标检测到：由 detect_sub_ 回调切换到 INTERCEPT
}

// ============================================================
//  状态机：INTERCEPT
//  基于视觉 PNG 导引冲向目标
// ============================================================
void VisionPNG::handle_intercept()
{
    arm();
    set_offboard_mode();

    // ---- 目标丢失分级处理 ----
    if (lost_frames_ > 0 && lost_frames_ < coast_thresh_) {
        // 短暂丢失（< coast_thresh_ 帧）：惯性续飞，保持上一帧速度+偏航
        // 同时继续旋转机头寻找目标（保持 d_yaw_ 不变）
        publish_offboard_velocity_mode();
        publish_velocity_setpoint(coast_vx_, coast_vy_, coast_vz_, (float)d_yaw_);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 100,
            "[INTERCEPT/COAST] 惯性续飞 lost=%d v=(%.1f,%.1f,%.1f)",
            lost_frames_, coast_vx_, coast_vy_, coast_vz_);
        return;
    }

    if (lost_frames_ >= lost_thresh_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
            "[INTERCEPT] 目标长时间丢失 lost_frames=%d，切换 TRACK_LOST", lost_frames_);
        state_ = VpngState::TRACK_LOST;
        return;
    }

    // ---- 正常拦截：计算速度指令 ----
    Eigen::Vector3d Vt(vx_, vy_, vz_);
    double d_v = Vt.norm() + d_gain_;
    d_v = std::min(d_v, speed_cmd_);
    d_v = std::max(d_v, 2.0);

    // 从 PNG 角度合成 NED 速度指令
    float vx_cmd = (float)(std::cos(d_v_angle_v_) * std::sin(d_v_angle_z_) * d_v);  // N
    float vy_cmd = (float)(std::cos(d_v_angle_v_) * std::cos(d_v_angle_z_) * d_v);  // E
    float vz_cmd = (float)(std::sin(d_v_angle_v_) * d_v);                            // D

    // ---- ey 垂直视场补偿（抵消前飞低头导致的相机下倾）----
    // ey < 0：目标在画面上方（图像坐标 y 轴向下，负值=目标偏上）
    //        → UAV 需要上升 → NED vz_cmd 减小（更负）
    //        → vz_ey = k_ey_ * ey_ < 0，叠加后 vz_cmd 变小（上升）✓
    // ey > 0：目标在画面下方 → UAV 需要下降 → vz_ey > 0，vz_cmd 增大（下降）✓
    // 补偿量限幅至 ±3 m/s
    {
        float vz_ey = (float)(k_ey_ * ey_);
        vz_ey = std::max(-3.0f, std::min(3.0f, vz_ey));
        vz_cmd += vz_ey;
    }

    // 缓存速度供惯性续飞使用
    coast_vx_  = vx_cmd;
    coast_vy_  = vy_cmd;
    coast_vz_  = vz_cmd;
    coast_yaw_ = (float)d_yaw_;

    publish_offboard_velocity_mode();
    publish_velocity_setpoint(vx_cmd, vy_cmd, vz_cmd, (float)d_yaw_);

    // ---- 统计：更新最近接距离 + 命中判定 ----
    save_stats_to_csv();

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 200,
        "[INTERCEPT] v_cmd=(%.2f,%.2f,%.2f) yaw_r=%.3f |V|=%.2f ex=%.0f ey=%.0f",
        vx_cmd, vy_cmd, vz_cmd, d_yaw_, Vt.norm(), ex_, ey_);
}

// ============================================================
//  状态机：TRACK_LOST
//  目标长时间丢失（> lost_thresh_ 帧），制动减速并原地慢旋寻找目标
// ============================================================
void VisionPNG::handle_track_lost()
{
    // 制动：发送零速度，但保持上一次的偏航旋转方向寻找目标
    // 若 d_yaw_ 为 0（已对中），则轻微旋转扫描
    float search_yaw = (std::abs(d_yaw_) > 0.01) ? (float)d_yaw_ : 0.2f;
    publish_offboard_velocity_mode();
    publish_velocity_setpoint(0.0f, 0.0f, 0.0f, search_yaw);

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "[TRACK_LOST] 减速旋转搜索目标... yaw_rate=%.2f", search_yaw);

    // 恢复逻辑由 detect_sub_ 回调处理
}

// ============================================================
//  状态机：DONE
//  拦截完成，悬停
// ============================================================
void VisionPNG::handle_done()
{
    publish_offboard_velocity_mode();
    publish_velocity_setpoint(0.0f, 0.0f, 0.0f, 0.0f);
    RCLCPP_INFO_ONCE(get_logger(), "★★★ 拦截完成，悬停中 ★★★");
}

// ============================================================
//  统计记录 —— 写 CSV（仅在 INTERCEPT 阶段调用）
//  使用 target_pos_stats_ 计算真实距离（导引算法不得调用此变量）
// ============================================================
void VisionPNG::save_stats_to_csv()
{
    // 延迟打开文件
    if (!csv_file_.is_open()) {
        csv_file_.open(csv_path_, std::ios::out);
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(get_logger(), "[统计] 无法打开 CSV 文件: %s", csv_path_.c_str());
            return;
        }
        csv_file_ << "time_s,"
                  << "self_x,self_y,self_z,"
                  << "target_x,target_y,target_z,"
                  << "dist_m,"
                  << "los_angle_v,los_angle_z,"
                  << "d_v_angle_v,d_v_angle_z,"
                  << "ex,ey,"
                  << "vx,vy,vz,"
                  << "detect_w,state\n";
        RCLCPP_INFO(get_logger(), "[统计] CSV 已创建: %s", csv_path_.c_str());
    }

    double ros_time = this->now().seconds();

    // 计算真实距离（统计用，导引不使用）
    float dist = -1.0f;
    bool just_hit = false; //标记是否在当前帧命中
    if (target_pos_ok_) {
        dist = (target_pos_stats_ - self_pos_stats_).norm();

        // 更新最近接距离
        if ((double)dist < stats_.min_distance) {
            stats_.min_distance = (double)dist;
        }

        // 命中判定（统计层）
        if (dist < hit_radius_ && !stats_.hit_recorded) {
            stats_.hit_recorded   = true;
            stats_.intercept_time = ros_time;
            just_hit = true; // 标记本帧为命中帧
            RCLCPP_WARN(get_logger(),
                "★ [统计] 检测到命中！真实距离=%.3f m (统计专用)", dist);
            state_ = VpngState::DONE;
        }
    }

    // 节流：每 10 帧（约 200ms @ 20Hz png_calculate）写一行，避免文件过大
    static int csv_counter = 0;
    csv_counter++;
    if ((csv_counter % 2 != 0) && !just_hit) {
            return; 
        }

    csv_file_ << std::fixed << std::setprecision(4)
              << ros_time << ","
              << self_pos_stats_(0) << "," << self_pos_stats_(1) << "," << -self_pos_stats_(2) << ","
              << target_pos_stats_(0) << "," << target_pos_stats_(1) << "," << -target_pos_stats_(2) << ","
              << dist << ","
              << LOS_angle_v_ << "," << LOS_angle_z_ << ","
              << d_v_angle_v_ << "," << d_v_angle_z_ << ","
              << ex_ << "," << ey_ << ","
              << vx_ << "," << vy_ << "," << vz_ << ","
              << detect_w_ << ","
              << static_cast<int>(state_) << "\n";
    csv_file_.flush();
}

// ============================================================
//  PX4 发布辅助函数
// ============================================================
void VisionPNG::arm()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

void VisionPNG::set_offboard_mode()
{
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
}

void VisionPNG::publish_offboard_position_mode()
{
    OffboardControlMode msg{};
    msg.position  = true;
    msg.velocity  = false;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    offboard_pub_->publish(msg);
}

void VisionPNG::publish_offboard_velocity_mode()
{
    OffboardControlMode msg{};
    msg.position  = false;
    msg.velocity  = true;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    offboard_pub_->publish(msg);
}

void VisionPNG::publish_position_setpoint(float x, float y, float z)
{
    TrajectorySetpoint msg{};
    msg.position  = {x, y, z};
    msg.yaw       = (float)yaw_;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    traj_pub_->publish(msg);
}

void VisionPNG::publish_velocity_setpoint(float vx, float vy, float vz, float yaw_rate)
{
    TrajectorySetpoint msg{};
    msg.position  = {std::nanf(""), std::nanf(""), std::nanf("")};
    msg.velocity  = {vx, vy, vz};
    msg.yaw       = std::nanf("");
    msg.yawspeed  = yaw_rate;
    msg.timestamp = get_clock()->now().nanoseconds() / 1000;
    traj_pub_->publish(msg);
}

void VisionPNG::publish_vehicle_command(uint16_t command, float p1, float p2)
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

// ============================================================
//  main
// ============================================================
int main(int argc, char* argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionPNG>());
    rclcpp::shutdown();
    return 0;
}

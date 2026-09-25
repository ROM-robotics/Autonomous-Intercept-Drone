#pragma once
/*
 * vision_png_control.hpp
 *
 * 基于视觉的比例导引（Vision-based Proportional Navigation Guidance）
 *
 * ┌─────────────────────────────────────────────────────────────────────┐
 * │  导引算法（纯视觉，不读目标位置）                                      │
 * │    输入：/camera_detect_result  (uav_common_msg/RectMsg)             │
 * │          /px4_1/fmu/out/vehicle_odometry   (自身姿态)                │
 * │          /px4_1/fmu/out/vehicle_local_position (自身速度/位置)        │
 * │    输出：/px4_1/fmu/in/trajectory_setpoint  (速度指令)               │
 * │          /px4_1/fmu/in/offboard_control_mode                        │
 * │          /px4_1/fmu/in/vehicle_command                              │
 * │          /vpng_data  (uav_common_msg/Data，导引过程数据，用于绘图)    │
 * ├─────────────────────────────────────────────────────────────────────┤
 * │  统计记录（仅用于数据分析，不参与导引计算）                             │
 * │    输入：/px4_2/fmu/out/vehicle_gps_position (目标真实GPS，统计专用)  │
 * │          /px4_1/fmu/out/vehicle_gps_position (自身GPS，统计专用)      │
 * │    输出：intercept_stats.csv（记录拦截全程数据）                       │
 * └─────────────────────────────────────────────────────────────────────┘
 *
 * 算法原理：
 *   1. 从像素坐标 (cx, cy) 计算 LOS 方位角与仰角：
 *        ex = cx - image_cx,  ey = cy - image_cy
 *        LOS_cam = [ex, ey, foc]  (相机坐标系)
 *        LOS_ned = R_b2n * R_c2b * LOS_cam
 *        LOS_angle_v = atan2(LOS_ned_z, sqrt(x²+y²))
 *        LOS_angle_z = atan2(LOS_ned_x, LOS_ned_y)
 *   2. 比例导引（PNG）：
 *        dσ_v = LOS_angle_v - last_LOS_angle_v
 *        dσ_z = LOS_angle_z - last_LOS_angle_z
 *        θ_v_cmd = N * dσ_v + last_θ_v
 *        θ_z_cmd = N * dσ_z + last_θ_z
 *   3. 期望速度：
 *        vx = cos(θ_v_cmd)*sin(θ_z_cmd) * speed   (NED North)
 *        vy = cos(θ_v_cmd)*cos(θ_z_cmd) * speed   (NED East)
 *        vz = sin(θ_v_cmd) * speed                 (NED Down)
 */

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <array>
#include <chrono>

// PX4 消息
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/hover_thrust_estimate.hpp>

// 自定义消息
#include <uav_common_msg/msg/rect_msg.hpp>
#include <uav_common_msg/msg/data.hpp>

// 地理坐标库（仅统计模块使用）
#include <GeographicLib/LocalCartesian.hpp>

// ============================================================
//  类型别名
// ============================================================
using OffboardControlMode  = px4_msgs::msg::OffboardControlMode;
using TrajectorySetpoint   = px4_msgs::msg::TrajectorySetpoint;
using VehicleCommand       = px4_msgs::msg::VehicleCommand;
using VehicleStatus        = px4_msgs::msg::VehicleStatus;
using VehicleOdometry      = px4_msgs::msg::VehicleOdometry;
using VehicleLocalPosition = px4_msgs::msg::VehicleLocalPosition;
using SensorGps            = px4_msgs::msg::SensorGps;
using HoverThrustEstimate  = px4_msgs::msg::HoverThrustEstimate;
using RectMsg              = uav_common_msg::msg::RectMsg;
using DataMsg              = uav_common_msg::msg::Data;

// ============================================================
//  状态机枚举
// ============================================================
enum class VpngState : uint8_t {
    TAKE_OFF,       // 起飞到待机高度，等待悬停推力收敛
    SEARCHING,      // 已到高度，等待视觉检测到目标
    INTERCEPT,      // 视觉 PNG 导引拦截中
    TRACK_LOST,     // 目标丢失，悬停保持
    DONE            // 拦截完成（命中目标）
};

// ============================================================
//  VisionPNG 节点
// ============================================================
class VisionPNG : public rclcpp::Node
{
public:
    VisionPNG();
    ~VisionPNG();

private:
    // ----------------------------------------------------------
    //  核心控制循环
    // ----------------------------------------------------------
    void control_loop();          ///< 200 Hz 姿态/发布循环
    void png_calculate();         ///< 20  Hz 视觉 PNG LOS 计算

    // ----------------------------------------------------------
    //  状态机各阶段处理
    // ----------------------------------------------------------
    void handle_takeoff();
    void handle_searching();
    void handle_intercept();
    void handle_track_lost();
    void handle_done();

    // ----------------------------------------------------------
    //  PX4 发布辅助
    // ----------------------------------------------------------
    void arm();
    void set_offboard_mode();
    void publish_offboard_position_mode();
    void publish_offboard_velocity_mode();
    void publish_offboard_att_mode();
    void publish_position_setpoint(float x, float y, float z);
    void publish_velocity_setpoint(float vx, float vy, float vz, float yaw_rate = 0.0f);
    void publish_vehicle_command(uint16_t command, float p1 = 0.0f, float p2 = 0.0f);

    // ----------------------------------------------------------
    //  统计 / CSV 记录（不参与导引）
    // ----------------------------------------------------------
    void save_stats_to_csv();

    // ----------------------------------------------------------
    //  定时器
    // ----------------------------------------------------------
    rclcpp::TimerBase::SharedPtr timer_control_;   ///< 200 Hz
    rclcpp::TimerBase::SharedPtr timer_png_;       ///< 20  Hz

    // ----------------------------------------------------------
    //  发布者
    // ----------------------------------------------------------
    rclcpp::Publisher<OffboardControlMode>::SharedPtr  offboard_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr   traj_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr       cmd_pub_;
    rclcpp::Publisher<DataMsg>::SharedPtr              data_pub_;   ///< /vpng_data 调试数据

    // ----------------------------------------------------------
    //  订阅者 —— 导引用
    // ----------------------------------------------------------
    rclcpp::Subscription<RectMsg>::SharedPtr            detect_sub_;        ///< 视觉检测结果（导引核心输入）
    rclcpp::Subscription<VehicleOdometry>::SharedPtr    odometry_sub_;      ///< 自身姿态
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr  local_pos_sub_;  ///< 自身速度/位置
    rclcpp::Subscription<VehicleStatus>::SharedPtr      status_sub_;        ///< 飞行状态
    rclcpp::Subscription<HoverThrustEstimate>::SharedPtr hover_sub_;        ///< 悬停推力估计

    // ----------------------------------------------------------
    //  订阅者 —— 纯统计用（导引算法禁止使用这些数据）
    // ----------------------------------------------------------
    rclcpp::Subscription<SensorGps>::SharedPtr self_gps_sub_;    ///< 自身 GPS（统计坐标建立）
    rclcpp::Subscription<SensorGps>::SharedPtr target_gps_sub_;  ///< 目标 GPS（仅统计）

    // ===========================================================
    //  ██  导引相关成员变量  ██
    // ===========================================================

    // ---- 相机内参（IMX214 @ OAK-D Lite，SDF: hFOV=1.204rad, 1920x1080）----
    // focal = (1920/2) / tan(1.204/2) = 960 / tan(0.602) = 1397.2 px
    double focal_length_  = 1397.2; ///< 焦距（像素）
    int    image_width_   = 1920;   ///< 图像宽度（像素）
    int    image_height_  = 1080;   ///< 图像高度（像素）

    // 相机→机体坐标系旋转矩阵 R_c2b
    // 与 los_control.cpp 保持一致：Cx→By, Cy→Bz, Cz→Bx
    Eigen::Matrix3d R_c2b_;

    // ---- 最新检测结果 ----
    int detect_x_      = 0;   ///< bbox 左上角 x（像素）
    int detect_y_      = 0;   ///< bbox 左上角 y（像素）
    int detect_w_      = -1;  ///< bbox 宽度；-1 = 目标丢失
    int detect_h_      = 0;   ///< bbox 高度
    bool detect_fresh_ = false;  ///< 是否有新检测帧

    // 连续丢失帧计数与阈值
    int lost_frames_   = 0;
    int lost_thresh_   = 45;   ///< 超过此值切换到 TRACK_LOST（原地搜索）
    int coast_thresh_  = 15;   ///< 低于此值时惯性续飞（约 0.5s@30Hz）

    // ---- 自身状态（导引使用）----
    double roll_  = 0.0;
    double pitch_ = 0.0;
    double yaw_   = 0.0;

    float vx_ = 0.0f;   ///< 自身速度 NED（来自 VehicleLocalPosition）
    float vy_ = 0.0f;
    float vz_ = 0.0f;

    float local_z_ = 0.0f;  ///< 自身 NED z 高度

    float hover_thrust_    = 0.5f;
    bool  hover_thrust_ok_ = false;
    bool  offboard_active_ = false;

    // ---- PNG 导引状态 ----
    double LOS_angle_v_      = 0.0;  ///< 当前俯仰 LOS 角
    double LOS_angle_z_      = 0.0;  ///< 当前方位 LOS 角
    double last_LOS_angle_v_ = 0.0;
    double last_LOS_angle_z_ = 0.0;

    double d_v_angle_v_  = 0.0;   ///< 期望速度仰角
    double d_v_angle_z_  = 0.0;   ///< 期望速度方位角
    double last_v_angle_v_ = 0.0;
    double last_v_angle_z_ = 0.0;

    Eigen::Vector3d LOS_vec_;       ///< 当前 LOS 向量（NED）
    Eigen::Vector3d last_LOS_vec_;  ///< 上一帧 LOS 向量（用于检测重复帧）

    double ex_ = 0.0;  ///< 像素误差 x（相对图像中心）
    double ey_ = 0.0;  ///< 像素误差 y

    // PNG 参数
    double N_png_    = 4.0;   ///< 比例导引系数（导航增益）
    double speed_cmd_ = 5.0; ///< 期望冲击速度（m/s）
    double d_gain_   = 2.0;   ///< 速度增量增益（与 los_control 一致）
    double kv_       = 4.0;   ///< 俯仰 PNG 增益（与 los_control kv 对应）
    double kz_       = 4.0;   ///< 方位 PNG 增益（与 los_control kz 对应）

    // 偏航跟踪控制（主动 Yaw 跟踪，保持目标在视场中心）
    // d_yaw = k1_yaw * ex + k2_yaw * d(ex)  [rad/s]
    // 图像宽 1920px，中心960px，最大误差960px
    // 期望最大偏航率约 0.8 rad/s → k1_yaw ≈ 0.8/960 ≈ 0.0008
    double k1_yaw_  = 0.0008;   ///< 像素误差比例项增益（rad/s per pixel）
    double k2_yaw_  = 0.0002;   ///< 像素误差微分项增益（rad/s per pixel/frame）
    double d_yaw_   = 0.0;      ///< 当前偏航角速率指令（rad/s）
    double last_ex_ = 0.0;

    // 垂直视场补偿增益：抵消前飞低头导致相机下倾
    // ey<0 目标在画面上方（图像坐标 y 向下）→ vz_ey=k_ey*ey<0 → vz_cmd 减小 → UAV 上升
    // 图像高 1080px，最大误差 540px，期望最大补偿 3 m/s
    // k_ey = 3.0 / 540 ≈ 0.0056 m/s per pixel
    double k_ey_    = 0.0056;   ///< ey→vz 垂直补偿增益（m/s per pixel）

    // 惯性续飞缓存（目标丢失时保持上一帧速度指令继续飞行）
    float coast_vx_ = 0.0f;  ///< 惯性续飞 vx 缓存
    float coast_vy_ = 0.0f;  ///< 惯性续飞 vy 缓存
    float coast_vz_ = 0.0f;  ///< 惯性续飞 vz 缓存
    float coast_yaw_= 0.0f;  ///< 惯性续飞 yawspeed 缓存

    bool png_initialized_ = false;

    // 起飞参数
    double standby_altitude_  = -6.0;  ///< NED z（负值 = 上升）
    int    takeoff_counter_   = 0;
    int    takeoff_frames_    = 500;   ///< 起飞预热帧数（5ms × 500 = 2.5s）

    // 状态机
    VpngState state_ = VpngState::TAKE_OFF;

    // 时间戳（毫秒）
    int64_t detect_timestamp_  = 0;  ///< 最新检测帧时间
    int64_t control_timestamp_ = 0;  ///< 控制循环时间
    int64_t time_diff_         = 0;  ///< 检测与控制的时间差（ms）

    // ===========================================================
    //  ██  统计记录成员变量（不参与导引）██
    // ===========================================================
    // 坐标系（以 px4_1 首次 GPS 为原点）
    GeographicLib::LocalCartesian lc_stats_;
    bool lc_stats_init_ = false;

    Eigen::Vector3f self_pos_stats_   = {0, 0, 0};  ///< 自身位置（NED，统计用）
    Eigen::Vector3f target_pos_stats_ = {0, 0, 0};  ///< 目标位置（NED，统计用，导引禁用）
    bool target_pos_ok_ = false;

    // 命中判定半径（统计用）
    float hit_radius_ = 0.8f;

    // CSV 文件
    std::ofstream csv_file_;
    std::string   csv_path_ = "/home/verser/ros2_ws/vpng_intercept_stats.csv";

    // 拦截统计摘要
    struct InterceptStats {
        int    total_detect_frames  = 0;  ///< 总检测帧数
        int    lost_frames_total    = 0;  ///< 累计丢失帧数
        double min_distance         = 1e9;///< 最近接距离（m）
        double intercept_time       = -1.0;///< 命中时刻（ROS 时间 s）
        bool   hit_recorded         = false;
    } stats_;
};

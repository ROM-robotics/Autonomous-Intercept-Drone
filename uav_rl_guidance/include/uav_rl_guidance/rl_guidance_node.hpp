#pragma once
/*
 * rl_guidance_node.hpp
 *
 * 基于强化学习 (GRU 策略) 的视觉制导拦截节点
 * ── uav_vision_png 的可替换实现
 *
 * 导引算法（纯视觉，严禁使用目标 GPS）:
 *   /camera_detect_result → 特征(15维) → GRU策略 → 速度指令
 *   watchdog 异常 → 回退内置 PNG
 *
 * 统计记录（仅用于数据分析，不参与导引计算）:
 *   订阅 px4_2 GPS → 计算真实距离 → CSV
 *
 * 定时器: 50Hz 主控制循环（心跳+状态机）
 *         20Hz 策略推理（与训练 dt=0.05 一致）
 */

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <onnxruntime_cxx_api.h>
#include <fstream>
#include <string>
#include <optional>
#include <tuple>
#include <array>
#include <cmath>
#include <memory>

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
enum class RLState : uint8_t {
    TAKE_OFF,       // 起飞到待机高度
    SEARCHING,      // 已到高度，等待视觉检测
    INTERCEPT,      // 策略/PNG 导引拦截中
    TRACK_LOST,     // 目标长时间丢失，悬停搜索
    DONE            // 拦截完成（命中目标）
};

// ============================================================
//  RLGuidanceNode
// ============================================================
class RLGuidanceNode : public rclcpp::Node
{
public:
    RLGuidanceNode();
    ~RLGuidanceNode();

private:
    // ---- 控制循环 ----
    void control_loop();          // 50 Hz: heartbeat + state machine
    void policy_tick();           // 20 Hz: GRU inference + action decode + watchdog

    // ---- 状态机各阶段 ----
    void handle_takeoff();
    void handle_searching();
    void handle_intercept_heartbeat();   // 50 Hz heartbeat
    void handle_intercept_policy();      // 20 Hz policy (called from policy_tick)
    void handle_track_lost();
    void handle_done();

    // ---- 策略推理 (ONNX Runtime) ----
    // obs: [1, 15]  h: [1, 1, 128]  →  (action: [1, 4], h_new: [1, 1, 128])
    std::pair<std::array<float, 4>, std::array<float, 128>> policy_inference(
        const std::array<float, 15>& obs, const std::array<float, 128>& h);

    // ---- 特征构造 ----
    // det: (x, y, w, h) 或 nullopt; dt: 时间步长; 返回15维观测
    std::array<float, 15> build_features(
        const std::optional<std::tuple<int, int, int, int>>& det,
        float roll, float pitch, float yaw,
        float vx, float vy, float vz, float local_z, float dt);

    // ---- 动作解码 ----
    struct VelocityCmd { float vx, vy, vz, yaw_rate; };
    VelocityCmd decode_action(const std::array<float, 4>& action,
                              float los_v, float los_z);

    // ---- PNG 回退控制器 ----
    struct PngResult {
        float vx, vy, vz, yaw_rate;
        float v_angle_v, v_angle_z;
        float los_v, los_z, ex, ey;
    };
    PngResult png_step(
        const std::optional<std::tuple<int, int, int, int>>& det,
        float roll, float pitch, float yaw,
        float vx, float vy, float vz);
    void png_reset();

    // ---- 几何辅助（静态）----
    static float wrap_pi(float angle);
    static void quat_to_euler(float q0, float q1, float q2, float q3,
                              float& roll, float& pitch, float& yaw);
    static Eigen::Matrix3f euler_to_r_b2n(float roll, float pitch, float yaw);
    static void pixel_to_los(float ex, float ey, float focal,
                             float roll, float pitch, float yaw,
                             float& los_v, float& los_z);
    static void angles_to_velocity(float v_angle_v, float v_angle_z, float speed,
                                   float& vx, float& vy, float& vz);

    // ---- PX4 发布辅助 ----
    void arm();
    void set_offboard_mode();
    void publish_offboard_position_mode();
    void publish_offboard_velocity_mode();
    void publish_position_setpoint(float x, float y, float z);
    void publish_velocity_setpoint(float vx, float vy, float vz, float yaw_rate);
    void publish_vehicle_command(uint16_t command, float p1 = 0.0f, float p2 = 0.0f);
    void publish_debug_data(float los_v, float los_z, float ex, float ey);

    // ---- CSV 统计 ----
    void open_csv();
    void save_stats_to_csv(bool just_hit = false);

    // ---- 回调 ----
    void detect_cb(const RectMsg::SharedPtr msg);
    void odom_cb(const VehicleOdometry::SharedPtr msg);
    void local_pos_cb(const VehicleLocalPosition::SharedPtr msg);
    void status_cb(const VehicleStatus::SharedPtr msg);
    void hover_cb(const HoverThrustEstimate::SharedPtr msg);
    void self_gps_cb(const SensorGps::SharedPtr msg);
    void target_gps_cb(const SensorGps::SharedPtr msg);

    // ==================== 成员变量 ====================

    // ---- 定时器 ----
    rclcpp::TimerBase::SharedPtr timer_control_;  // 50 Hz
    rclcpp::TimerBase::SharedPtr timer_policy_;   // 20 Hz

    // ---- 发布者 ----
    rclcpp::Publisher<OffboardControlMode>::SharedPtr  offboard_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr   traj_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr       cmd_pub_;
    rclcpp::Publisher<DataMsg>::SharedPtr              data_pub_;  // /vpng_data

    // ---- 订阅者（导引用）----
    rclcpp::Subscription<RectMsg>::SharedPtr             detect_sub_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr     odom_sub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr  local_pos_sub_;
    rclcpp::Subscription<VehicleStatus>::SharedPtr       status_sub_;
    rclcpp::Subscription<HoverThrustEstimate>::SharedPtr hover_sub_;

    // ---- 订阅者（纯统计用，导引算法禁止使用）----
    rclcpp::Subscription<SensorGps>::SharedPtr self_gps_sub_;
    rclcpp::Subscription<SensorGps>::SharedPtr target_gps_sub_;

    // ---- 状态机 ----
    RLState state_ = RLState::TAKE_OFF;

    // ---- 检测结果 ----
    std::optional<std::tuple<int, int, int, int>> detect_;  // (x,y,w,h) or nullopt
    int lost_frames_  = 0;
    int coast_thresh_ = 30;   // 惯性续飞帧数 (at detect rate)
    int lost_thresh_  = 90;   // 切换到 TRACK_LOST 帧数

    // ---- 自身状态 ----
    float roll_ = 0, pitch_ = 0, yaw_ = 0;
    float vx_ = 0, vy_ = 0, vz_ = 0;
    float local_z_ = 0;
    bool  hover_thrust_ok_ = false;
    bool  offboard_active_  = false;

    // ---- 起飞 ----
    float standby_altitude_ = -6.0f;
    int   takeoff_counter_  = 0;
    int   takeoff_frames_   = 125;   // 50Hz × 2.5s

    // ---- 策略状态 (ONNX Runtime) ----
    std::array<float, 128> gru_hidden_;   // GRU hidden state [1, 1, 128]
    int watchdog_count_ = 0;          // remaining latch frames
    int watchdog_total_ = 0;          // total watchdog triggers
    bool fallback_png_  = false;      // pure PNG baseline mode
    bool bench_test_    = false;      // skip takeoff for bench test

    // ONNX Runtime 成员
    Ort::Env ort_env_;
    Ort::SessionOptions ort_session_opts_;
    std::unique_ptr<Ort::Session> ort_session_;
    Ort::AllocatorWithDefaultOptions ort_allocator_;
    Ort::MemoryInfo ort_memory_info_{nullptr};
    std::string ort_input_name_obs_;
    std::string ort_input_name_h_;
    std::string ort_output_name_action_;
    std::string ort_output_name_h_;

    // ---- 特征构造器状态 ----
    bool  has_los_ = false;
    float feat_los_v_ = 0, feat_los_z_ = 0;
    float log_w_ = std::log(20.0f), log_h_ = std::log(20.0f);
    float feat_age_ = 0;
    float feat_ex_ = 0, feat_ey_ = 0;

    // ---- PNG 回退状态 ----
    bool  png_init_ = false;
    float png_d_angle_v_ = 0, png_d_angle_z_ = 0;
    float png_last_los_v_ = 0, png_last_los_z_ = 0;
    float png_last_v_angle_v_ = 0, png_last_v_angle_z_ = 0;
    float png_last_ex_ = 0;
    float png_d_yaw_ = 0;

    // ---- 惯性续飞缓存 ----
    float coast_vx_ = 0, coast_vy_ = 0, coast_vz_ = 0, coast_yaw_ = 0;
    std::string last_cmd_source_ = "png";

    // ---- 导引/相机参数 ----
    float focal_length_  = 1397.2f;
    int   image_width_   = 1920;
    int   image_height_  = 1080;
    float kv_ = 4.0f, kz_ = 4.0f;
    float speed_cmd_ = 5.0f, speed_min_ = 2.0f;
    float d_gain_   = 1.0f;
    float k1_yaw_   = 0.0005f, k2_yaw_ = 0.0002f;
    float k_ey_     = 0.012f;
    float yaw_rate_max_  = 1.0f;
    float los_diff_thresh_ = 0.02f;
    float elev_clamp_   = M_PI / 4.0f;
    float search_yaw_rate_ = 0.2f;
    float vz_ey_max_  = 3.0f;

    // ---- 相机→机体旋转矩阵 (静态常量) ----
    static const Eigen::Matrix3f R_c2b_;

    // ---- 动作解码参数（从 policy_meta.json 加载，可被 params.yaml 覆盖）----
    float dv_angle_max_ = 1.2f;

    // WGS84 近似换算常数
    static constexpr double kMPerDegLat = 111320.0;

    // ==================== 统计成员（不参与导引）====================

    // GPS → NED 变换
    double gps_origin_lat_ = 0, gps_origin_lon_ = 0, gps_origin_alt_ = 0;
    bool   gps_origin_set_ = false;

    Eigen::Vector3f self_pos_stats_   = {0, 0, 0};
    Eigen::Vector3f target_pos_stats_ = {0, 0, 0};
    bool target_pos_ok_ = false;

    float  hit_radius_    = 0.8f;
    double min_distance_  = 1e9;
    bool   hit_recorded_  = false;
    int    total_detect_frames_ = 0;
    int    lost_frames_total_   = 0;
    int    csv_counter_  = 0;
    std::ofstream csv_file_;
    std::string   csv_path_ = "/home/verser/ros2_ws/rl_intercept_stats.csv";

    void gps_to_ned(double lat, double lon, double alt,
                    double& n, double& e, double& d);
};

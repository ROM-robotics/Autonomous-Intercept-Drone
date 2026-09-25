#ifndef PNG_CONTROL_HPP
#define PNG_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>

#include <GeographicLib/LocalCartesian.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <cmath>

#include <fstream>
#include <iomanip>

using namespace px4_msgs::msg;
using namespace std::chrono_literals;

enum class InterceptState { INIT, TAKEOFF, INTERCEPT, DONE };

class PngInterceptor : public rclcpp::Node
{
public:
    PngInterceptor();
    ~PngInterceptor() = default;

private:
    // -------- 主控制循环 --------
    void control_loop();

    void handle_init();
    void handle_takeoff();
    void handle_intercept();

    // -------- 比例导引（与 IBVS 代码相同的角度更新公式）--------
    Eigen::Vector3f compute_png_velocity(const Eigen::Vector3f& target_pos_ned);

    // -------- PX4 发布辅助 --------
    void arm();
    void set_offboard_mode();
    void publish_offboard_velocity_mode();
    void publish_offboard_position_mode();
    void publish_velocity_setpoint(float vx, float vy, float vz, float yaw);
    void publish_position_setpoint(float x, float y, float z, float yaw);
    void publish_vehicle_command(uint16_t command, float p1 = 0.0f, float p2 = 0.0f);

    // -------- 发布者 --------
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr  traj_pub_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr      cmd_pub_;

    // -------- 订阅者 --------
    rclcpp::Subscription<SensorGps>::SharedPtr           self_gps_sub_;   // px4_1 绝对位置
    rclcpp::Subscription<SensorGps>::SharedPtr           target_gps_sub_; // px4_2 绝对位置
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr self_vel_sub_;   // px4_1 速度
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr target_vel_sub_; // px4_2 速度
    rclcpp::Subscription<VehicleStatus>::SharedPtr        status_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // -------- 状态 --------
    InterceptState state_{InterceptState::INIT};
    bool offboard_active_{false};
    bool status_received_{false};
    int  nav_state_{-1};     // PX4 nav_state 原始值（14=OFFBOARD）
    int  arming_state_{-1};  // PX4 arming_state 原始值（2=ARMED）
    int  init_counter_{0};

    // 自身位置（NED，相对于自身第一个GPS点）
    Eigen::Vector3f self_pos_{0.0f, 0.0f, 0.0f};
    Eigen::Vector3f self_vel_{0.0f, 0.0f, 0.0f};
    bool self_pos_ok_{false};
    float local_z_{0.0f};             // 本地高度 (VehicleLocalPosition.z)

    // 目标位置（NED，转换到自身坐标系后）
    Eigen::Vector3f target_pos_{0.0f, 0.0f, 0.0f};
    Eigen::Vector3f target_vel_{0.0f, 0.0f, 0.0f};
    bool target_pos_ok_{false};

    // 坐标系：以自身第一个GPS为原点的 LocalCartesian
    GeographicLib::LocalCartesian lc_;
    bool lc_initialized_{false};

    // -------- PNG 状态（角度更新法，与 IBVS 保持一致）--------
    bool  png_initialized_{false};
    float last_LOS_angle_v_{0.0f};  // LOS 仰角（上一帧）
    float last_LOS_angle_z_{0.0f};  // LOS 方位角（上一帧）
    float d_v_angle_v_{0.0f};       // 期望速度仰角
    float d_v_angle_z_{0.0f};       // 期望速度方位角
    float last_v_angle_v_{0.0f};    // 上一帧速度仰角
    float last_v_angle_z_{0.0f};    // 上一帧速度方位角
    float target_yaw_{0.0f};        // 目标方位角（用于机头指向目标）

    // -------- 导引参数 --------
    const float N_{4.0f};               // 导引比
    const float intercept_speed_{8.0f}; // 最大拦截速度 m/s
    const float takeoff_alt_{-5.0f};    // 起飞 NED 高度（与目标一致）
    const float hit_radius_{0.5f};      // 判定命中距离 m
    const float dt_{0.05f};             // 控制周期 20Hz

    // -------- 数据记录 --------
    std::ofstream csv_file_;
    GeographicLib::LocalCartesian geo_converter_; // 统一的坐标转换器
    bool origin_set_ = false;
    void save_data_to_csv();
};

#endif // PNG_CONTROL_HPP

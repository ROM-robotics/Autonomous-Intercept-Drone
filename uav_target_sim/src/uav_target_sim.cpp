#include "uav_target_sim.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include "px4_msgs/msg/vehicle_rates_setpoint.hpp"

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <algorithm>
#include <random>
#include <px4_msgs/msg/sensor_gps.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;


UavTargetControl::UavTargetControl() : Node("uav_target_sim")
{
    // --- 参数声明与解析 ---
    this->declare_parameter<std::string>("motion_mode", "circle");
    this->declare_parameter<double>("max_range", 10.0);

    std::string mode_str = this->get_parameter("motion_mode").as_string();
    max_range_ = static_cast<float>(this->get_parameter("max_range").as_double());

    if (mode_str == "circle")           motion_mode_ = MotionMode::CIRCLE;
    else if (mode_str == "sinusoidal")  motion_mode_ = MotionMode::SINUSOIDAL;
    else if (mode_str == "random_walk") motion_mode_ = MotionMode::RANDOM_WALK;
    else {
        RCLCPP_WARN(this->get_logger(),
            "Unknown motion_mode '%s', defaulting to 'circle'", mode_str.c_str());
        motion_mode_ = MotionMode::CIRCLE;
    }

    RCLCPP_INFO(this->get_logger(), "=== Motion mode: %s | max_range: %.1fm ===",
        mode_str.c_str(), max_range_);

    // --- 发布者配置 ---
    offboard_control_mode_publisher_ =
        this->create_publisher<OffboardControlMode>("/px4_2/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ =
        this->create_publisher<TrajectorySetpoint>("/px4_2/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ =
        this->create_publisher<VehicleCommand>("/px4_2/fmu/in/vehicle_command", 10);

    // --- 订阅者配置 ---
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    vehicle_rates_setpoint_subscription_ = this->create_subscription<VehicleRatesSetpoint>(
        "/px4_2/fmu/out/vehicle_rates_setpoint", qos,
        [this](const VehicleRatesSetpoint::SharedPtr msg) { (void)msg; });

    global_position_sub_ = this->create_subscription<SensorGps>(
        "/px4_2/fmu/out/vehicle_gps_position", qos,
        std::bind(&UavTargetControl::global_position_callback, this, std::placeholders::_1));

    // --- 状态变量初始化 ---
    offboard_setpoint_counter_ = 0;
    theta_ = 0.0f;
    angular_speed_ = 0.05f;

    pos_x_ = 5.0f;
    pos_y_ = 0.0f;
    pos_z_ = -5.0f;
    vel_x_ = 0.0f;
    vel_y_ = 0.0f;
    sim_time_ = 0.0f;

    if (motion_mode_ == MotionMode::SINUSOIDAL) {
        vel_x_ = base_speed_;
    }

    // --- 定时器 ---
    timer_ = this->create_wall_timer(100ms, [this]() {
        publish_offboard_control_mode();

        if (offboard_setpoint_counter_ < 15) {
            publish_trajectory_setpoint(5.0, 0.0, -5.0);
            offboard_setpoint_counter_++;
        }
        else if (offboard_setpoint_counter_ == 15) {
            RCLCPP_INFO(this->get_logger(), "Switching to Offboard and Arming...");
            this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            this->arm();
            offboard_setpoint_counter_++;
        }
        else {
            execute_motion();
        }
    });
}

void UavTargetControl::arm() {
    publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void UavTargetControl::publish_offboard_control_mode() {
    OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

void UavTargetControl::publish_trajectory_setpoint(float x, float y, float z) {
    TrajectorySetpoint msg{};
    msg.position = {x, y, z};
    msg.yaw = 0.0;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

// ============================================================
//  边界约束：超出 max_range_ 时施加回复力
//  目标靠近边界会被平滑拉回，保证不飞出拦截机视野
// ============================================================
void UavTargetControl::apply_boundary() {
    float dx = pos_x_ - boundary_center_x_;
    float dy = pos_y_ - boundary_center_y_;
    float dist = std::sqrt(dx * dx + dy * dy);

    if (dist > max_range_ * 0.8f && dist > 0.01f) {
        // 从80%处开始施加渐进回复力
        float overshoot = (dist - max_range_ * 0.8f) / (max_range_ * 0.2f);
        float restore = 3.0f * overshoot * overshoot;
        float nx = dx / dist;
        float ny = dy / dist;
        vel_x_ -= restore * nx * dt_;
        vel_y_ -= restore * ny * dt_;
    }
}

// ============================================================
//  运动模式调度
// ============================================================
void UavTargetControl::execute_motion() {
    switch (motion_mode_) {
        case MotionMode::CIRCLE:       move_in_circle();    break;
        case MotionMode::SINUSOIDAL:   move_sinusoidal();   break;
        case MotionMode::RANDOM_WALK:  move_random_walk();  break;
    }
}

// ============================================================
//  1. 匀速圆周运动 (原有, 天然有界无需约束)
// ============================================================
void UavTargetControl::move_in_circle() {
    theta_ += angular_speed_;
    if (theta_ > 2 * M_PI) theta_ -= 2 * M_PI;

    float x = radius_ * std::cos(theta_);
    float y = radius_ * std::sin(theta_);

    TrajectorySetpoint msg{};
    msg.position = {x, y, -5.0};
    msg.yaw = theta_;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

// ============================================================
//  2. 正弦机动: a_T = A*sin(ωt), A=0.5m/s², ω=0.5rad/s
//     前向匀速 + 侧向正弦加速度 + 边界约束
// ============================================================
void UavTargetControl::move_sinusoidal() {
    sim_time_ += dt_;

    float a_y = sin_amplitude_ * std::sin(sin_omega_ * sim_time_);
    vel_y_ += a_y * dt_;

    apply_boundary();

    pos_x_ += vel_x_ * dt_;
    pos_y_ += vel_y_ * dt_;

    TrajectorySetpoint msg{};
    msg.position = {pos_x_, pos_y_, pos_z_};
    msg.yaw = std::atan2(vel_y_, vel_x_);
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

// ============================================================
//  3. 随机游走: v_T(t+Δt) = v_T(t) + N(0, σ_v), σ_v=0.2m/s
//     无规律逃逸 + 边界约束
// ============================================================
void UavTargetControl::move_random_walk() {
    vel_x_ += sigma_v_ * normal_dist_(rng_);
    vel_y_ += sigma_v_ * normal_dist_(rng_);

    float speed = std::sqrt(vel_x_ * vel_x_ + vel_y_ * vel_y_);
    if (speed > max_speed_) {
        vel_x_ *= max_speed_ / speed;
        vel_y_ *= max_speed_ / speed;
    }

    apply_boundary();

    pos_x_ += vel_x_ * dt_;
    pos_y_ += vel_y_ * dt_;

    TrajectorySetpoint msg{};
    msg.position = {pos_x_, pos_y_, pos_z_};
    msg.yaw = std::atan2(vel_y_, vel_x_);
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
}

// ============================================================
//  Vehicle command & GPS callback
// ============================================================
void UavTargetControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 3;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

void UavTargetControl::global_position_callback(const px4_msgs::msg::SensorGps::SharedPtr msg)
{
    (void)msg;
    if (!msg) {
        RCLCPP_WARN(this->get_logger(), "Received null GPS message");
        return;
    }
}

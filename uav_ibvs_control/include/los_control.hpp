#ifndef LOS_CONTROL_H
#define LOS_CONTROL_H

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/input_rc.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include "px4_msgs/msg/vehicle_rates_setpoint.hpp"
#include "px4_msgs/msg/hover_thrust_estimate.hpp"

#include "uav_common_msg/msg/rect_msg.hpp"
#include "uav_common_msg/msg/data.hpp"

#include "ctime"
#include <chrono>
#include <iostream>
#include <GeographicLib/LocalCartesian.hpp>
#include "mutex"
//  数学库
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

using namespace Eigen;
using namespace px4_msgs::msg;
using namespace std::chrono;
//  状态机枚举
enum class State { take_off, image_detect, tracking, detcte_loss };

class uav_chase : public rclcpp::Node
{
public:
    uav_chase();
    ~uav_chase();
private: 
	//  解锁无人机
	void arm();
	//	锁定无人机
	void disarm();
	//	发布控制模式
	void publish_offboard_control_mode();
	//  发布轨迹控制
	void publish_trajectory_setpoint(float x,float y,float z,float yaw);
	//  发布速度控制
	void publish_trajectory_velocity(float x,float y,float z,float yaw);
	//  发布控制命令
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	//  发布升力及角度控制
	void publish_rates_setpoint(double roll, double pitch, double yaw, float thrust);
    //  发布升力控制模式
	void publish_offboard_control_mode_att();
    //  发布速度控制模式
	void publish_offboard_control_mode_vel();

    //	主任务函数
	void missionfunction();

    //  起飞任务函数
	void take_off();
	//	等待识别
	void waiting_for_detect();
	//	追踪主体
	void tracker();
	//  追踪方法
	void LOS_function();
	//	悬停方法
	void hover();


    //  分解向量 将向量分解为沿着法线和垂直法线部分
	void decomposeVector(const Eigen::Vector3d& vector2de, const Eigen::Vector3d& LOS,Eigen::Vector3d& vector_parallel,Eigen::Vector3d& vector_perpendicular);
    
    //  LOS控制函数
	void LOS_calculate();
    //  叉乘
	Eigen::Matrix3d cross_product(const Eigen::Vector3d& V);
    //  矩阵转向量
	Eigen::Vector3d vex(const Eigen::Matrix3d& M);

    //  姿态环定时器
    rclcpp::TimerBase::SharedPtr timer_Attitude_loop;
	//  速度环定时器
    rclcpp::TimerBase::SharedPtr timer_LOS_loop;
    
    //  发布方
    //  发布控制模式
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	//  发布轨迹控制
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    //  发布控制命令
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	//	发布姿态以及升力数据
	rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr vehicle_rates_setpoint_publisher_;
	//  发布绘图数据
	rclcpp::Publisher<uav_common_msg::msg::Data>::SharedPtr data_plot_publisher_;

	//  订阅者
	//	无人机状态订阅者
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr VehicleState_subscription_;
	//	GPS订阅者
	rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr GPS_subscription_;
	//	追踪数据订阅
	rclcpp::Subscription<uav_common_msg::msg::RectMsg>::SharedPtr track_result_subscription_;
	//	姿态数据订阅
	rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_subscription_;
	//	RC遥控器数据订阅
	//	rclcpp::Subscription<InputRc>::SharedPtr RC_data_subscription_;
	//	速度数据订阅
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr local_position_subscription_;
	//	升力订阅者
	rclcpp::Subscription<VehicleRatesSetpoint>::SharedPtr vehicle_rates_setpoint_subscription_;
	//	悬浮升力订阅者
	rclcpp::Subscription<HoverThrustEstimate>::SharedPtr hover_thrust_subscription_;

	rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr gps_sensor_subscription_;


	double init_xyz[3] = {0, 0, 0};
	bool xyz_initialized = false;
    
    //  LOS控制参数
    double foc=544.0;   //焦距
    double image_width=1920;
	double image_height=1080;
    //  FOV保持器 X轴
    double k1=0.003;
    double k2=0.002;
    //  FOV保持器 Y轴
    double d_gain=4;
    double kv=7;
    double kz=7;
    double m=0.945;
    float standby_height = 1;

    //传感器读取的数据
    double x=0;
	double y=0;
	double uav_width=0;
	double uav_height=0;
    double roll=0;
	double pitch=0;
	double yaw=0;
    double vx=0;
	double vy=0;
	double vz=0;
    double hover_thrust=0.5;

    
    //状态变量
    double lost_track=1;
	bool tracke_msg_flag = false;
	bool uav_msg_flag = false;
	bool start_flag = true;
	double track_time = 0;
	bool offboard_state = false;
	long long track_timestamp;
	long long control_timestamp;
	long long time_diff;
	double local_position_z = 0;
    //	LOS标志位 是否初次进入LOS控制
	bool LOS_flag = false;	
    //  升力计算是否完成
	bool hover_flag = false;
    //  控制次数
	double control_time = 0;
    //  控制丢失次数
	double control_lose_time = 0;
    //	时间标志
	double test_time=0;
    //  时间戳函数
    std::atomic<uint64_t> timestamp_;
	std::atomic<uint64_t> old_track_timestamp_;
	std::atomic<uint64_t> new_track_timestamp_; 
    //	任务状态机变量
	State state = State::take_off;
    
    //LOS计算过程中使用到的变量
    // FOV保持器相关变量
    double d_yaw = 0;
	double ex = 0;
	double ey = 0;
	double last_ex = 0;
    // 添加用于存储角度数据的容器
	std::vector<double> los_angle_v_data;
	std::vector<double> los_angle_z_data;
	std::vector<double> vt_angle_v_data;
	std::vector<double> vt_angle_z_data;
	std::vector<double> d_vt_angle_v_data;
	std::vector<double> d_vt_angle_z_data;
    // 输出的速度控制的变量
    double d_vx = 0;
	double d_vy = 0;
	double d_vz = 0;

    // PNG算法部分
    // 速度的角度变量
    double v_angle_v = 0;
	double v_angle_z = 0;
    // 期望速度角度变量
    double d_v_angle_v = 0;
	double d_v_angle_z = 0;
    // LOS角度变量
	double LOS_angle_v = 0;
	double LOS_angle_z = 0;

    Eigen::Vector3d Nt;
	Eigen::Vector3d last_Nt;
	//	获取速度向量
	Eigen::Vector3d Vt;

	Eigen::Vector3d d_a;
	Eigen::Vector3d d_thrust;
	Eigen::Vector3d d_g = Eigen::Vector3d(0, 0, 9.81);
	Eigen::Vector3d d_thrust_direction;

	Eigen::Vector3d last_d_thrust_direction;
	//	单位矩阵
	Eigen::Matrix3d I3=Eigen::Matrix3d::Identity();
	Eigen::Vector3d rotationAxis;
	Eigen::Matrix3d d_rotation_M;
	Eigen::Vector3d d_w;
	Eigen::Vector3d body_z_axis = Eigen::Vector3d(0, 0, -1);
	double rotationAngle=0;
	Eigen::Vector3d d_Vt = Eigen::Vector3d::Zero();
	Eigen::Vector3d v_parallelpar;
	Eigen::Vector3d v_perpendicular;
	Eigen::Vector3d a_parallel;
	Eigen::Vector3d a_perpendicular;

	Eigen::AngleAxisd b2nrotationYaw;
    Eigen::AngleAxisd b2nrotationPitch;
    Eigen::AngleAxisd b2nrotationRoll;
	Eigen::Matrix3d b2nrotationMatrix;

	Eigen::Matrix3d c2brotation = (Eigen::Matrix3d() << 
                                    0, 0, 1,
                                    1, 0, 0,
                                    0, 1, 0).finished();
    
	double last_LOS_angle_v = 0;
	double last_LOS_angle_z = 0;
	double last_v_angle_z = 0;
	double last_v_angle_v = 0;
	double d_v = 0;
	double real_thrust = 0;
	float thrust_proportion = 0;
	int offboard_setpoint_counter_;  
};
/**
 * @brief cross product function
 */
Eigen::Matrix3d uav_chase::cross_product(const Eigen::Vector3d& V)
{
	Eigen::Matrix3d M;
	M << 0, -V(2), V(1),
		V(2), 0, -V(0),
		-V(1), V(0), 0;
	return M;
}
/**
 * @brief vex function
 */
Eigen::Vector3d uav_chase::vex(const Eigen::Matrix3d& M)
{
	Eigen::Vector3d V;
	V << M(2,1), M(0,2), M(1,0);
	return V;
}
/**
 * @brief Send a command to control w and f
 */
void uav_chase::publish_rates_setpoint(double roll, double pitch, double yaw, float thrust)
{
    // 创建消息实例
	VehicleRatesSetpoint message{};
    message.roll = roll;
    message.pitch = pitch;
    message.yaw = yaw;
    message.thrust_body = {0,0,thrust};
    vehicle_rates_setpoint_publisher_->publish(message);
}
/**
 * @brief Send a command to Arm the vehicle
 */
void uav_chase::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}
/**
 * @brief Send a command to Disarm the vehicle
 */
void uav_chase::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}
/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void uav_chase::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	//	启用位置控制
	msg.position = true;
	//	禁用速度控制
	msg.velocity = false;
	//	禁用加速度控制
	msg.acceleration = false;
	//	禁用姿态控制
	msg.attitude = false;
	//	禁用机体角速度控制
	msg.body_rate = false;

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}
/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void uav_chase::publish_offboard_control_mode_vel()
{
	OffboardControlMode msg{};
	//	启用位置控制
	msg.position = false;
	//	禁用速度控制
	msg.velocity = true;
	//	禁用加速度控制
	msg.acceleration = false;
	//	禁用姿态控制
	msg.attitude = false;
	//	禁用机体角速度控制
	msg.body_rate = false;

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}
/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void uav_chase::publish_offboard_control_mode_att()
{
	OffboardControlMode msg{};
	//	启用速度控制
	msg.position = false;
	//	禁用速度控制
	msg.velocity = false;
	//	禁用加速度控制
	msg.acceleration = false;
	//	禁用姿态控制
	msg.attitude = false;
	//	禁用机体角速度控制
	msg.body_rate = true;

	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}
/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void uav_chase::publish_trajectory_setpoint(float x,float y,float z,float yaw)
{
	TrajectorySetpoint msg{};
	//	使用的是FRD坐标系
	msg.position = {x,y,z};
	msg.yaw = yaw; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}
/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void uav_chase::publish_trajectory_velocity(float x,float y,float z,float yaw)
{
	TrajectorySetpoint msg{};
	//	使用的是FRD坐标系
	msg.position = {nan(""),nan(""),nan("")};
	msg.velocity = {x,y,z};
	msg.yaw = nan(""); // [-PI:PI]
	msg.yawspeed = yaw;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}
/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
//  发布无人机命令
void uav_chase:: publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 2;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

#endif
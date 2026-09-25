#include <los_control.hpp>
#include <fstream>
#include <iomanip>
#include <GeographicLib/Geocentric.hpp>
using namespace GeographicLib;

uav_chase::~uav_chase()
{
	std::cout<<"mission finished"<<std::endl;
}

uav_chase::uav_chase():Node("uav_ibvs_control")
{
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        //创建控制模式发布者
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/px4_1/fmu/in/offboard_control_mode", 10);
        //创建轨迹点模式发布者
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/px4_1/fmu/in/trajectory_setpoint", 10);
        //控制命令发布者
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/px4_1/fmu/in/vehicle_command", 10);
        //创建轨迹及升力控制
        vehicle_rates_setpoint_publisher_ = this->create_publisher<VehicleRatesSetpoint>("/px4_1/fmu/in/vehicle_rates_setpoint", 10);
        //创建绘图数据发布者
        data_plot_publisher_ = this->create_publisher<uav_common_msg::msg::Data>("/los_data", 10);

        //创建悬浮升力监听者
        hover_thrust_subscription_ = this->create_subscription<HoverThrustEstimate>("/px4_1/fmu/out/hover_thrust_estimate", qos,
        [this](const HoverThrustEstimate::SharedPtr msg)
        {
            if(msg->hover_thrust_var<0.0025)
            {this->hover_thrust = msg->hover_thrust;
            hover_flag = true;}
        });
        //	创建 识别信息话题接收者
        track_result_subscription_ = this->create_subscription<uav_common_msg::msg::RectMsg>("/camera_detect_result", 10,
        [this](const uav_common_msg::msg::RectMsg::UniquePtr msg)
        {
            this->x = msg->x;
            this->y = msg->y;
            this->uav_width = msg->width;
            this->uav_height = msg->height;
            this->tracke_msg_flag = true;
            track_time +=1;
            if(msg->width == -1)
            {
                lost_track+=1;
            }else
            {
                lost_track = 0;
                state = State::tracking;
            }

            auto now = std::chrono::system_clock::now();
            track_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        });
        //	创建 姿态信息话题接收者
        vehicle_odometry_subscription_ = this->create_subscription<VehicleOdometry>("/px4_1/fmu/out/vehicle_odometry", qos,
        [this](const VehicleOdometry::SharedPtr msg)
        {
            double q0, q1, q2, q3;
            q0 = msg->q[0];
            q1 = msg->q[1];
            q2 = msg->q[2];
            q3 = msg->q[3];
            this->roll  = std::atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2));
            this->pitch = std::asin(2 * (q0 * q2 - q3 * q1));
            this->yaw = std::atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
			std::cout<<"roll: "<<roll<<" pitch: "<<pitch<<" yaw: "<<yaw<<std::endl;
        });
        //	创建 速度信息话题接收者
        local_position_subscription_ = this->create_subscription<VehicleLocalPosition>("/px4_1/fmu/out/vehicle_local_position", qos,
        [this](const VehicleLocalPosition::SharedPtr msg)
        {
            this->vx = msg->vx;
            this->vy = msg->vy;
            this->vz = msg->vz;
            local_position_z = msg->z;
        });
        //	创建 无人机状态话题接受者
        VehicleState_subscription_ = this->create_subscription<VehicleStatus>("/px4_1/fmu/out/vehicle_status", qos,
        [this](const VehicleStatus::SharedPtr msg)
        {
            if(msg->nav_state == msg->NAVIGATION_STATE_OFFBOARD)
            {
                this->offboard_state = true;
            }
        });




		gps_sensor_subscription_ = this->create_subscription<px4_msgs::msg::SensorGps>(
		"/px4_2/fmu/out/vehicle_gps_position", qos,
		[this](const px4_msgs::msg::SensorGps::UniquePtr msg) {

			static std::ofstream log_file;
			if (!log_file.is_open()) {
				log_file.open("/home/verser/ros2_ws/px4_2_xyz_log.txt", std::ios::out | std::ios::app);
				if (!log_file.is_open()) {
					RCLCPP_ERROR(this->get_logger(), "无法打开日志文件！");
					return;
				}
				log_file << "timestamp, world_x, world_y, world_z\n";
			}

			Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());
			double cur_x, cur_y, cur_z;
			earth.Forward(msg->latitude_deg, msg->longitude_deg, msg->altitude_msl_m,
						cur_x, cur_y, cur_z);

			if (!this->xyz_initialized) {
				this->init_xyz[0] = cur_x;
				this->init_xyz[1] = cur_y;
				this->init_xyz[2] = cur_z;
				this->xyz_initialized = true;
			}

			double world_x = cur_x - this->init_xyz[0];
			double world_y = cur_y - this->init_xyz[1];
			double world_z = cur_z - this->init_xyz[2];

			std::cout << "world xyz: x=" << world_x
					<< " y=" << world_y
					<< " z=" << world_z << std::endl;

			double timestamp = this->now().seconds();
			log_file << std::fixed << std::setprecision(6)
					<< timestamp << ", "
					<< world_x  << ", "
					<< world_y  << ", "
					<< world_z  << "\n";
			log_file.flush();
		});

    }
    //  前置速度环计算  20Hz
    auto los_loop_callback = [this]() -> void {
        if(lost_track<1)
        {LOS_calculate();}
    };
    timer_LOS_loop = this->create_wall_timer(20ms, los_loop_callback);

    //  姿态环计算	5Hz
    auto attitude_loop_callback = [this]() -> void {
        missionfunction();
    };
    timer_Attitude_loop = this->create_wall_timer(5ms, attitude_loop_callback);
}
/**
 * @brief missionfunction
 */
void uav_chase::missionfunction()
{
	// 获取当前时间点
	auto now = std::chrono::system_clock::now();
	control_timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
	time_diff = control_timestamp - track_timestamp;
	
	switch (state)
	{
	case State::take_off:
		take_off();
		break;
	case State::image_detect: 
		waiting_for_detect();
		break;
	case State::tracking:
		tracker();
		break;
	case State::detcte_loss:
		hover();
		break;
	default:
		break;
	}
}
/**
 * @brief calculate desire velocity
 */
void uav_chase::LOS_calculate()
{
    //	计算LOS向量
	ex =  x+uav_width/2 - image_width/2;
	ey =  y+uav_height/2 - image_height/2;
	Nt << ex,ey,foc;

	// 获得从BCS旋转至ENU坐标系下的旋转矩阵
	// 绕Z轴旋转
	b2nrotationYaw   =  Eigen::AngleAxisd(yaw, Eigen::Vector3d(0, 0, 1));
	// 绕Y轴旋转
	b2nrotationPitch =  Eigen::AngleAxisd(pitch, Eigen::Vector3d(0, 1, 0));
	// 绕X轴旋转
    b2nrotationRoll  =  Eigen::AngleAxisd(roll, Eigen::Vector3d(1, 0, 0));
	// 合成旋转矩阵：先偏航（yaw），后俯仰（pitch），再滚转（roll）
    b2nrotationMatrix = b2nrotationYaw.toRotationMatrix() 
								* b2nrotationPitch.toRotationMatrix() 
								* b2nrotationRoll.toRotationMatrix();
	
	// 获得从CCS旋转至BCS坐标系下的矩阵 


	//	计算LOS的v平面和z平面角度
	//	将LOS矢量从CCS旋转至BCS坐标系
	Nt = Nt;
	Nt = c2brotation * Nt;
	//	将LOS矢量从BCS旋转至NED坐标系
	Nt = b2nrotationMatrix * Nt;

	LOS_angle_v = atan2(Nt[2],sqrt(Nt[1]*Nt[1]+Nt[0]*Nt[0]));
	LOS_angle_z = atan2(Nt[0],Nt[1]);

	double diff_LOS_angle_v = LOS_angle_v-last_LOS_angle_v;
	double diff_LOS_angle_z = LOS_angle_z-last_LOS_angle_z;
	// printf("diff_LOS_angle_v: %f\n",LOS_angle_v-last_LOS_angle_v);
	// printf("diff_LOS_angle_z: %f\n",LOS_angle_z-last_LOS_angle_z);

	control_time += 1;
	if ((last_LOS_angle_v==0&&last_LOS_angle_z==0)||Vt.norm()<1)
	{
		d_v_angle_v = LOS_angle_v;
		d_v_angle_z = LOS_angle_z;

		last_LOS_angle_v = LOS_angle_v;
		last_LOS_angle_z = LOS_angle_z;

		last_v_angle_v = atan2(Vt[2],sqrt(Vt[1]*Vt[1]+Vt[0]*Vt[0]));
		last_v_angle_z = atan2(Vt[0],Vt[1]);
	
		last_Nt = Nt;
	}else
	{	
		// 检测是否与上一刻的相同
		if(Nt == last_Nt)
		{
			control_lose_time += 1;
		}
		last_Nt = Nt;
		if(abs(diff_LOS_angle_v)<0.02)
		{

		}
		else
		{
			last_v_angle_v = atan2(Vt[2],sqrt(Vt[1]*Vt[1]+Vt[0]*Vt[0]));
			last_v_angle_z = atan2(Vt[0],Vt[1]);

			// d_v_angle_v = kv*(LOS_angle_v - last_LOS_angle_v) + LOS_angle_v;
			// d_v_angle_z = kz*(LOS_angle_z - last_LOS_angle_z) + LOS_angle_z;

			d_v_angle_v = kv*(LOS_angle_v - last_LOS_angle_v) + last_v_angle_v;
			d_v_angle_z = kz*(LOS_angle_z - last_LOS_angle_z) + last_v_angle_z;
			
			// }
			// d_v_angle_v = kv*(LOS_angle_v - last_LOS_angle_v) + d_v_angle_v;
			// d_v_angle_z = kz*(LOS_angle_z - last_LOS_angle_z) + d_v_angle_z;
			// filter_diff_angle_v = filter_diff_angle_v+(1-filter_diff_v)*diff_LOS_angle_v;
			// filter_diff_angle_z = filter_diff_angle_z+(1-filter_diff_z)*diff_LOS_angle_z;

			// 计算当前速度角度
			printf("--------------------data-------------------\n");
			printf("last_v_angle_v: %f\n",last_v_angle_v);
			printf("d_v_angle_v: %f\n",d_v_angle_v);
			printf("LOS_angle_v: %f\n",LOS_angle_v);

			last_LOS_angle_v = LOS_angle_v;
			last_LOS_angle_z = LOS_angle_z;
			// last_v_angle_v = d_v_angle_v;
			// last_v_angle_z = d_v_angle_z;

			uav_common_msg::msg::Data data_msg{};
			data_msg.d_v_angle_v = d_v_angle_v;
			data_msg.d_v_angle_z = d_v_angle_z;
			data_msg.v_angle_v = atan2(Vt[2], sqrt(Vt[1]*Vt[1]+Vt[0]*Vt[0]));
			data_msg.v_angle_z = atan2(Vt[0], Vt[1]);
			data_msg.los_angle_v = LOS_angle_v;
			data_msg.los_angle_z = LOS_angle_z;
			data_msg.diff_los_angle_v = diff_LOS_angle_v;
			data_msg.diff_los_angle_z = diff_LOS_angle_z;
			data_msg.ex = ex;
			data_msg.ey = ey;
			data_plot_publisher_->publish(data_msg);
		}
	}

	// 使用LOS向量作为速度方向
	// d_Vt << cos(LOS_angle_v)*sin(LOS_angle_z), 
    //     cos(LOS_angle_v)*cos(LOS_angle_z), 
    //     sin(LOS_angle_v);

	
	d_Vt << cos(d_v_angle_v)*sin(d_v_angle_z), 
         cos(d_v_angle_v)*cos(d_v_angle_z), 
         sin(d_v_angle_v);
		 
	//	速度模长增益
	d_v = Vt.norm()+d_gain;

		//	计算FOV保持器
	if(last_ex == 0)
	{	d_yaw = 0;
		last_ex = ex;}
	else{d_yaw = k1*ex+k2*(ex-last_ex);
		last_ex = ex;}
	d_Vt = d_Vt*d_v;
}

void uav_chase::LOS_function()
{
    Vt << vx,vy,vz;
	// Attitude loop 姿态环控制
	// 获取期望加速度 Vt为NED坐标系 d_Vt为NED坐标系
	d_a =(d_Vt-Vt);
	d_g << 0,0,9.81;
	// 获取期望升力	 d_g为NED坐标系
	
	// 获得从BCS旋转至ENU坐标系下的旋转矩阵
	// 绕Z轴旋转
	Eigen::AngleAxisd b2nrotationYaw(yaw, Eigen::Vector3d(0, 0, 1)); 
	// 绕Y轴旋转
	Eigen::AngleAxisd b2nrotationPitch(pitch, Eigen::Vector3d(0, 1, 0)); 
	// 绕X轴旋转
    Eigen::AngleAxisd b2nrotationRoll(roll, Eigen::Vector3d(1, 0, 0)); 
	// 合成旋转矩阵：先偏航（yaw），后俯仰（pitch），再滚转（roll）
    Eigen::Matrix3d b2nrotationMatrix = b2nrotationYaw.toRotationMatrix() 
								* b2nrotationPitch.toRotationMatrix() 
								* b2nrotationRoll.toRotationMatrix();
	
	// 获得从CCS旋转至BCS坐标系下的矩阵
	// 绕Z轴旋转
	Eigen::Matrix3d c2brotation;
	c2brotation << 	0, 0, 1,  // Cx -> By
                	1, 0, 0,  // Cy -> Bz
                 	0, 1, 0;  // Cz -> Bx

	d_thrust = (d_a-d_g)*200;
	
	// 获取期望升力的方向向量 NED
	d_thrust_direction = d_thrust/d_thrust.norm();
	// 计算上次的升力方向 NED
	last_d_thrust_direction = b2nrotationMatrix*body_z_axis;
	//	获取旋转轴
	rotationAxis = last_d_thrust_direction.cross(d_thrust_direction);
	//	获取旋转轴的旋转角度
	rotationAngle = acos(last_d_thrust_direction.dot(d_thrust_direction));
	//	计算r的叉乘矩阵
	Eigen::Matrix3d r_cross_matrix = cross_product(rotationAxis);
	//	获取Rd
	d_rotation_M = (I3+r_cross_matrix*sin(rotationAngle)+r_cross_matrix*r_cross_matrix*(1-cos(rotationAngle)))*b2nrotationMatrix;
	//	计算角速度
	Eigen::Matrix3d temp = d_rotation_M.transpose()*b2nrotationMatrix-b2nrotationMatrix.transpose()*d_rotation_M;

	d_w = vex(temp);
	d_w = -1*d_w;
	d_w[2] = d_w[2]+d_yaw;
	// d_w[0] = 0;
	if(d_w.norm()>8)
	{d_w = d_w/d_w.norm()*8;}

	// real_thrust = std::min(std::max(last_d_thrust_direction.dot((d_a - m*d_g)),0.0),(m*9.81/hover_thrust));
	real_thrust = std::min(std::max(last_d_thrust_direction.dot((d_a - m*d_g)),0.0),(m*9.81/hover_thrust));

	thrust_proportion = real_thrust/(m*9.81/hover_thrust);
	
	if(Vt.norm()>1)
	{
		publish_offboard_control_mode_att();
		publish_rates_setpoint(d_w[0],d_w[1],d_w[2],-thrust_proportion);
	}
	else
	{
		publish_offboard_control_mode_vel();
		publish_trajectory_velocity(d_Vt[0],d_Vt[1],d_Vt[2],d_yaw);
	}

	// publish_offboard_control_mode_att();
	// publish_rates_setpoint(d_w[0],d_w[1],d_w[2],-thrust_proportion);

	// publish_offboard_control_mode_vel();
	// publish_trajectory_velocity(d_Vt[0],d_Vt[1],d_Vt[2],d_yaw);

	// publish_offboard_control_mode();
	// publish_trajectory_setpoint(0,0,-standby_height,-3.14);
}
/**
 * @brief take_off function
 */
void uav_chase::take_off()
{
    if (offboard_setpoint_counter_ == 500) {
	this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
	this->arm();}
	// SITL仿真
	this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
	this->arm();
	
	publish_offboard_control_mode();
	// publish_trajectory_setpoint(0,0,-standby_height,-3.14);
	publish_trajectory_setpoint(0, 0, -standby_height, this->yaw);

	if (offboard_setpoint_counter_ < 501) {
		offboard_setpoint_counter_++;
	}
	// 切换状态
	else{ 
		if (hover_flag)
		{state = State::image_detect;}
		else
		{
			publish_offboard_control_mode();
			// publish_trajectory_setpoint(0,0,-standby_height,-3.14);
			publish_trajectory_setpoint(0, 0, -standby_height, this->yaw);
		}
	}
}
void uav_chase::hover()
{ 
    // 悬停
	publish_offboard_control_mode_vel();
	publish_trajectory_velocity(0,0,0,this->yaw);
}
void uav_chase::waiting_for_detect()
{
    // 如果丢失目标，进入到速度控制模式，保证不坠机 track_time要超过100的原因是识别部分前100帧不稳定
	if(lost_track > 60 || track_time <100){
		publish_offboard_control_mode_vel();
		 publish_trajectory_velocity(0,0,0,this->yaw);

		// publish_offboard_control_mode();
		// publish_trajectory_setpoint(0,0,-standby_height,-3.14);
	}
}

void uav_chase::tracker()
// 如果控制时间小于500ms则进入控制模式
{
    if(time_diff < 500)
    {
        //	如果没有丢失目标则进入控制模式
        if(lost_track<1)
        {this->LOS_function();}
        else if(lost_track<30)
        {
        //丢失目标小于30帧但是大于1帧则可能误识别，继续冲击
            if(Vt.norm()>1)
            {
                publish_offboard_control_mode_att();
                publish_rates_setpoint(d_w[0],d_w[1],d_w[2],-thrust_proportion);
            }
            else
            {
                publish_offboard_control_mode_vel();
                publish_trajectory_velocity(d_Vt[0],d_Vt[1],d_Vt[2],d_yaw);
            }
        // publish_offboard_control_mode_vel();
        // publish_trajectory_velocity(d_Vt[0],d_Vt[1],d_Vt[2],d_yaw);
        }
        else
        {std::cout<<"lost track!!!"<<std::endl;
        state = State::detcte_loss;
        }
    }
}

int main(int argc, char *argv[]){
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<uav_chase>());
    rclcpp::shutdown();
    return 0;
}
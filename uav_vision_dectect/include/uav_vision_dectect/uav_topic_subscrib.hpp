#pragma once

#include <opencv2/core/mat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>


#include "LightTrack.h"
#include <px4_msgs/msg/sensor_gps.hpp>

#include "uav_common_msg/msg/rect_msg.hpp"

#include "yolo_detector.hpp"


class UavTopicSubscrib : public rclcpp::Node
{

    public:
        cv::Mat uav_camera_frame;
        cv::Rect uav_result_rect;
        UavTopicSubscrib();
        // Destructor - အရင်းအမြစ်များကို ဖယ်ရှားသည်
        ~UavTopicSubscrib();


    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr uav_image_sub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_position_sub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
        rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr gps_position_sub_;


        rclcpp::Publisher<uav_common_msg::msg::RectMsg>::SharedPtr uav_detect_result_publisher_;
        
        
        uav_common_msg::msg::RectMsg pub_uav_result_rect;

        cv_bridge::CvImagePtr orig_cv_ptr;


        /************************LightTrack ခြေရာခံခြင်း အပိုင်း************************/
        cv::Rect trackWindow;
        cv::Mat init_window;

        LightTrack *siam_tracker;
        int light_track_flag = 0;

        /************************YOLO Detector အရင်းအမြစ် အဖွဲ့ဝင် ကိန်းရှင်များ************************/

        // YOLO Detector ကို အစပြုသတ်မှတ်သည့် function
        void initTensorRT();

        std::unique_ptr<YoloDetector> yolo_detector_;


        /**************************************************/
        std::thread uav_detect_result_thread_;

        void uav_detect_result_loop();
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void global_position_callback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
        void cxy_wh_2_rect(const cv::Point& pos, const cv::Point2f& sz, cv::Rect &rect);
};




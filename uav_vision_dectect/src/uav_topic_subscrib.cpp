//
// Created by verse on 24-10-9.
//
#include "uav_topic_subscrib.hpp"
#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>
#include <memory> 


// 定义模型路径 (建议改为 ROS2 参数加载)
const std::string YOLO_ENGINE_PATH = "src/uav_vision_dectect/model/yolov5/GDUT_UAV.onnx";

void UavTopicSubscrib::initTensorRT()
{
    // 配置并初始化 YOLO 检测器
    DetectorConfig config;
    config.modelPath = YOLO_ENGINE_PATH;
    config.confThreshold = 0.4f;
    config.iouThreshold = 0.45f;
    config.inputWidth = 640;
    config.inputHeight = 640;

    try {
        RCLCPP_INFO(this->get_logger(), "Initializing YOLO Detector from: %s", config.modelPath.c_str());
        yolo_detector_ = std::make_unique<YoloDetector>(config);
        RCLCPP_INFO(this->get_logger(), "YOLO Detector Initialized Successfully.");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize YOLO Detector: %s", e.what());
    }
}

UavTopicSubscrib::~UavTopicSubscrib() 
{
    if (uav_detect_result_thread_.joinable()) {
        uav_detect_result_thread_.join();
    }
    if (siam_tracker) {
        delete siam_tracker;
        siam_tracker = nullptr;
    } 
}

UavTopicSubscrib::UavTopicSubscrib() : Node("uav_vision_dectect")
{
    /***********************************局部跟踪器初始化***********************************/
    std::string init_model = "/home/verser/ros2_ws/src/uav_vision_dectect/model/light_track/lighttrack_init";
    std::string update_model = "/home/verser/ros2_ws/src/uav_vision_dectect/model/light_track/lighttrack_update";

    siam_tracker = new LightTrack(init_model.c_str(), update_model.c_str());

    /***********************************初始化TensorRT***********************************/
    initTensorRT();


    /***********************************话题订阅***********************************/
    uav_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image",
        10,
        std::bind(&UavTopicSubscrib::image_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "------------uav_topic_subscrib------------");

    uav_result_rect = cv::Rect(-1, -1, -1, -1);

    uav_detect_result_publisher_ = this->create_publisher<uav_common_msg::msg::RectMsg>("/camera_detect_result", 10);
    uav_detect_result_thread_ = std::thread(&UavTopicSubscrib::uav_detect_result_loop, this);
}

void UavTopicSubscrib::uav_detect_result_loop()
{
    rclcpp::WallRate loop_rate(60);

    while (rclcpp::ok())
    {
        pub_uav_result_rect.header = std_msgs::msg::Header();
        pub_uav_result_rect.header.stamp = this->now(); // 建议加上时间戳
        pub_uav_result_rect.x = uav_result_rect.x;
        pub_uav_result_rect.y = uav_result_rect.y;
        pub_uav_result_rect.width = uav_result_rect.width;
        pub_uav_result_rect.height = uav_result_rect.height;
        pub_uav_result_rect.depth = 0;

        // 降低日志频率，避免刷屏
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
        //     "Detect_result: %d %d %d %d", uav_result_rect.x, uav_result_rect.y, uav_result_rect.width, uav_result_rect.height);
        
        uav_detect_result_publisher_->publish(pub_uav_result_rect);

        loop_rate.sleep();
    }
}

void UavTopicSubscrib::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // ===========================
    // 1. 图像转换 (cv_bridge)
    // ===========================
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // 浅拷贝/深拷贝处理
    cv::Mat frame = cv_ptr->image.clone(); // 建议 clone 一份，避免多线程修改 uav_camera_frame 冲突
    uav_camera_frame = frame; // 如果其他线程只读，可以直接赋值引用


    // ===========================
    // 2. 模式控制
    // ===========================
    // true: 丢失目标时用YOLO重识别，识别后启动跟踪; false: 纯YOLO检测
    bool enable_tracking = true; // TODO: 建议改为 ROS Param

    cv::Rect result_rect(-1, -1, -1, -1);      
    bool target_found = false; 
    std::string status_text = "";
    cv::Scalar color = cv::Scalar(0, 255, 0);

    double t = (double)cv::getTickCount();

    // ===========================
    // 3. 核心逻辑分支
    // ===========================
    bool need_yolo_detection = (!enable_tracking) || (enable_tracking && light_track_flag == 0);

    // 检查检测器是否初始化成功
    if (need_yolo_detection && yolo_detector_)
    {

        // 使用封装好的 YoloDetector 进行推理 ---
        std::vector<Detection> results = yolo_detector_->detect(frame);

        // 寻找最佳目标 (置信度最高)
        float best_score = 0;
        cv::Rect best_rect;
        bool has_valid_detection = false;

        for (const auto& det : results) 
        {
            if (det.conf > best_score) 
            {
                best_score = det.conf;
                best_rect = det.box;
                has_valid_detection = true;
            }
        }

        // --- 检测结果处理 ---
        if (has_valid_detection && best_score > 0.8) // 0.3 为业务逻辑的过滤阈值，可调
        {
            cv::Rect safe_rect = best_rect & cv::Rect(0, 0, frame.cols, frame.rows);
            
            // 检查框的有效性
            if (safe_rect.width > 0 && safe_rect.height > 0 && safe_rect.area() >= 10) 
            {
                if (!enable_tracking)
                {
                    // [模式A: 仅检测]
                    result_rect = safe_rect;
                    target_found = true;
                    status_text = "YOLO Detect (Score: " + std::to_string(best_score).substr(0, 4) + ")";
                    color = cv::Scalar(0, 0, 255); // 红色框
                    light_track_flag = 0; 
                }
                else
                {
                    // [模式B: 跟踪初始化]
                    light_track_flag = 1; 
                    this->trackWindow = safe_rect;
                    
                    Bbox box;
                    box.x0 = safe_rect.x;
                    box.y0 = safe_rect.y;
                    box.x1 = safe_rect.x + safe_rect.width;
                    box.y1 = safe_rect.y + safe_rect.height;

                    std::cout << ">>> Tracker Init Start..." << std::endl;
                    siam_tracker->init(frame.data, box, frame.rows, frame.cols);
                    std::cout << ">>> Tracker Init Done!" << std::endl;
                    
                    result_rect = safe_rect;
                    target_found = true;
                    status_text = "Global Track Init";
                    color = cv::Scalar(255, 0, 0); // 蓝色框表示初始化
                }
            }
        }
    }

    // ===========================
    // 4. 跟踪逻辑
    // ===========================
    if (enable_tracking && light_track_flag == 1 && !need_yolo_detection)
    {
        siam_tracker->track(frame.data);

        cv::Rect rect;
        cxy_wh_2_rect(siam_tracker->target_pos, siam_tracker->target_sz, rect);

        cv::Rect safe_rect = rect & cv::Rect(0, 0, frame.cols, frame.rows);
        
        if (safe_rect.area() > 0 && siam_tracker->target_pos_change() == 0)
        {
            result_rect = safe_rect;
            target_found = true;
            status_text = "Tracking";
            color = cv::Scalar(0, 255, 0); // 绿色框
        }
        else
        {
            status_text = "Track Lost";
            light_track_flag = 0; // 丢失，下帧转回 YOLO
        }
    }

    // ===========================
    // 5. 结果更新与 PNP 解算
    // ===========================
    if (target_found)
    {
        uav_result_rect = result_rect;

        // 绘制 (这里保留你原本的绘制逻辑，因为你有根据状态变色的需求)
        // 也可以混合使用 YoloDetector::draw，但它颜色是固定的
        cv::rectangle(frame, result_rect, color, 2);
        cv::putText(frame, status_text, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);

    }
    else
    {
        uav_result_rect = cv::Rect(-1, -1, -1, -1);

        if(!status_text.empty()) {
             cv::putText(frame, status_text, cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
        }
    }

    // ===========================
    // 6. 显示与帧率
    // ===========================
    double fps = cv::getTickFrequency() / ((double)cv::getTickCount() - t);
    std::string frameLabel = "FPS: " + std::to_string(fps).substr(0, 5);
    cv::putText(frame, frameLabel, cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);

    cv::namedWindow("Result", cv::WINDOW_NORMAL);
    cv::resizeWindow("Result", 640, 640);
    cv::imshow("Result", frame);
    cv::waitKey(1);
}

// 辅助函数保持不变
void UavTopicSubscrib::cxy_wh_2_rect(const cv::Point& pos, const cv::Point2f& sz, cv::Rect &rect)
{
    rect.x = std::max(0, pos.x - int(sz.x / 2));
    rect.y = std::max(0, pos.y - int(sz.y / 2));
    rect.width = int(sz.x);
    rect.height = int(sz.y);
}
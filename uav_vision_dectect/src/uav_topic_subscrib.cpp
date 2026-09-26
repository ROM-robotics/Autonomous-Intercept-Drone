//
// Created by verse on 24-10-9.
//
#include "uav_topic_subscrib.hpp"
#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>
#include <memory> 


// Model လမ်းကြောင်းကို သတ်မှတ်သည် (ROS2 parameter အဖြစ် ပြောင်းသုံးရန် အကြံပြုသည်)
const std::string YOLO_ENGINE_PATH = "src/uav_vision_dectect/model/yolov5/GDUT_UAV.onnx";

void UavTopicSubscrib::initTensorRT()
{
    // YOLO detector ကို ပြင်ဆင်သတ်မှတ်ပြီး အစပြုသည်
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
    /***********************************ဒေသဆိုင်ရာ ခြေရာခံကိရိယာ အစပြုသတ်မှတ်ခြင်း***********************************/
    std::string init_model = "/home/verser/ros2_ws/src/uav_vision_dectect/model/light_track/lighttrack_init";
    std::string update_model = "/home/verser/ros2_ws/src/uav_vision_dectect/model/light_track/lighttrack_update";

    siam_tracker = new LightTrack(init_model.c_str(), update_model.c_str());

    /***********************************TensorRT အစပြုသတ်မှတ်ခြင်း***********************************/
    initTensorRT();


    /***********************************Topic Subscription***********************************/
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
        pub_uav_result_rect.header.stamp = this->now(); // အချိန်တံဆိပ် (timestamp) ထည့်သွင်းရန် အကြံပြုသည်
        pub_uav_result_rect.x = uav_result_rect.x;
        pub_uav_result_rect.y = uav_result_rect.y;
        pub_uav_result_rect.width = uav_result_rect.width;
        pub_uav_result_rect.height = uav_result_rect.height;
        pub_uav_result_rect.depth = 0;

        // log ထုတ်ပေးမှု အကြိမ်ရေကို လျှော့ချပြီး မျက်နှာပြင်ပြည့်သွားခြင်းကို ရှောင်ရှားသည်
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
        //     "Detect_result: %d %d %d %d", uav_result_rect.x, uav_result_rect.y, uav_result_rect.width, uav_result_rect.height);
        
        uav_detect_result_publisher_->publish(pub_uav_result_rect);

        loop_rate.sleep();
    }
}

void UavTopicSubscrib::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // ===========================
    // 1. ပုံရိပ် ပြောင်းလဲခြင်း (cv_bridge)
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

    // shallow copy/deep copy ကိုင်တွယ်ခြင်း
    cv::Mat frame = cv_ptr->image.clone(); // thread များစွာက uav_camera_frame ကို ပြင်ဆင်မှု ပဋိပက္ခမဖြစ်စေရန် clone တစ်ခု ပြုလုပ်ရန် အကြံပြုသည်
    uav_camera_frame = frame; // အခြား thread များက ဖတ်ရုံသာ ပြုလုပ်ပါက reference ကို တိုက်ရိုက်သတ်မှတ်နိုင်သည်


    // ===========================
    // 2. Mode ထိန်းချုပ်ခြင်း
    // ===========================
    // true: target ပျောက်ဆုံးသွားပါက YOLO ဖြင့်ပြန်မှတ်သားပြီး ခြေရာခံမှု စတင်သည်; false: YOLO detection သီးသန့်
    bool enable_tracking = true; // TODO: ROS Param အဖြစ် ပြောင်းသုံးရန် အကြံပြုသည်

    cv::Rect result_rect(-1, -1, -1, -1);      
    bool target_found = false; 
    std::string status_text = "";
    cv::Scalar color = cv::Scalar(0, 255, 0);

    double t = (double)cv::getTickCount();

    // ===========================
    // 3. အဓိက logic ခွဲခြမ်းမှု
    // ===========================
    bool need_yolo_detection = (!enable_tracking) || (enable_tracking && light_track_flag == 0);

    // detector အောင်မြင်စွာ အစပြုနိုင်မနိုင် စစ်ဆေးသည်
    if (need_yolo_detection && yolo_detector_)
    {

        // ပြင်ဆင်ပြီးသား YoloDetector ကို အသုံးပြုပြီး inference ပြုလုပ်သည် ---
        std::vector<Detection> results = yolo_detector_->detect(frame);

        // အကောင်းဆုံး target ကို ရှာဖွေသည် (confidence အမြင့်ဆုံး)
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

        // --- စစ်ဆေးမှုရလဒ် ကိုင်တွယ်ခြင်း ---
        if (has_valid_detection && best_score > 0.8) // 0.3 သည် business logic filter threshold ဖြစ်ပြီး ချိန်ညှိနိုင်သည်
        {
            cv::Rect safe_rect = best_rect & cv::Rect(0, 0, frame.cols, frame.rows);
            
            // box ၏ မှန်ကန်မှုကို စစ်ဆေးသည်
            if (safe_rect.width > 0 && safe_rect.height > 0 && safe_rect.area() >= 10) 
            {
                if (!enable_tracking)
                {
                    // [Mode A: detection သီးသန့်]
                    result_rect = safe_rect;
                    target_found = true;
                    status_text = "YOLO Detect (Score: " + std::to_string(best_score).substr(0, 4) + ")";
                    color = cv::Scalar(0, 0, 255); // အနီရောင် box
                    light_track_flag = 0; 
                }
                else
                {
                    // [Mode B: ခြေရာခံမှု အစပြုခြင်း]
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
                    color = cv::Scalar(255, 0, 0); // အပြာရောင် box သည် အစပြုခြင်းကို ဖော်ပြသည်
                }
            }
        }
    }

    // ===========================
    // 4. ခြေရာခံမှု logic
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
            color = cv::Scalar(0, 255, 0); // အစိမ်းရောင် box
        }
        else
        {
            status_text = "Track Lost";
            light_track_flag = 0; // ပျောက်ဆုံးသွားသည်၊ နောက် frame တွင် YOLO သို့ ပြန်ပြောင်းသည်
        }
    }

    // ===========================
    // 5. ရလဒ် update ပြုလုပ်ခြင်းနှင့် PNP တွက်ချက်ခြင်း
    // ===========================
    if (target_found)
    {
        uav_result_rect = result_rect;

        // ရေးဆွဲခြင်း (status အလိုက် အရောင်ပြောင်းရန် လိုအပ်ချက်ရှိသောကြောင့် မူရင်း ရေးဆွဲခြင်း logic ကို ဤနေရာတွင် ထားရှိသည်)
        // YoloDetector::draw ကိုလည်း ရောနှော အသုံးပြုနိုင်သော်လည်း၊ ၎င်း၏ အရောင်မှာ ပုံသေ ဖြစ်သည်
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
    // 6. ပြသခြင်းနှင့် frame rate
    // ===========================
    double fps = cv::getTickFrequency() / ((double)cv::getTickCount() - t);
    std::string frameLabel = "FPS: " + std::to_string(fps).substr(0, 5);
    cv::putText(frame, frameLabel, cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);

    cv::namedWindow("Result", cv::WINDOW_NORMAL);
    cv::resizeWindow("Result", 640, 640);
    cv::imshow("Result", frame);
    cv::waitKey(1);
}

// အကူအညီ function မပြောင်းလဲပါ
void UavTopicSubscrib::cxy_wh_2_rect(const cv::Point& pos, const cv::Point2f& sz, cv::Rect &rect)
{
    rect.x = std::max(0, pos.x - int(sz.x / 2));
    rect.y = std::max(0, pos.y - int(sz.y / 2));
    rect.width = int(sz.x);
    rect.height = int(sz.y);
}
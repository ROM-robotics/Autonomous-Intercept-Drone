#pragma once

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

// စစ်ဆေးတွေ့ရှိမှု ရလဒ် struct
struct Detection {
    cv::Rect box;
    float conf;
    int classId;
};

// စစ်ဆေးကိရိယာ ပြင်ဆင်သတ်မှတ်ချက် paramaters
struct DetectorConfig {
    float confThreshold = 0.4f;
    float iouThreshold = 0.45f;
    std::string modelPath;
    int inputWidth = 640;
    int inputHeight = 640;
};

class YoloDetector {
public:
    // Constructor
    explicit YoloDetector(const DetectorConfig& config);

    // Destructor
    ~YoloDetector() = default;

    // ကူးယူခြင်းကို တားမြစ်ထားပြီး ရွှေ့ခြင်းကိုသာ ခွင့်ပြုသည်
    YoloDetector(const YoloDetector&) = delete;
    YoloDetector& operator=(const YoloDetector&) = delete;

    // အဓိက စစ်ဆေးသည့် function: ပုံအား ထည့်သွင်းပြီး စစ်ဆေးမှုရလဒ်စာရင်းကို ပြန်ပေးသည်
    std::vector<Detection> detect(const cv::Mat& img);

    // static အကူအညီ function: စစ်ဆေးမှုရလဒ်များကို ပုံပေါ်တွင် ရေးဆွဲသည်
    static void draw(cv::Mat& img, const std::vector<Detection>& objects);

private:
    cv::Mat letterbox(const cv::Mat& img, float& ratio, int& pad_x, int& pad_y);
    void nms(std::vector<Detection>& dets, float nms_thresh);
    static float iou(const Detection& a, const Detection& b);

    // အဖွဲ့ဝင် ကိန်းရှင်များ
    DetectorConfig config_;
    Ort::Env env_;
    Ort::Session session_;
    Ort::AllocatorWithDefaultOptions allocator_;

    std::string input_name_;
    std::string output_name_;
};
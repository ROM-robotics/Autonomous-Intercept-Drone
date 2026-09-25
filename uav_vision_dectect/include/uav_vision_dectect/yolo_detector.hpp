#pragma once

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

// 检测结果结构体
struct Detection {
    cv::Rect box;
    float conf;
    int classId;
};

// 检测器配置参数
struct DetectorConfig {
    float confThreshold = 0.4f;
    float iouThreshold = 0.45f;
    std::string modelPath;
    int inputWidth = 640;
    int inputHeight = 640;
};

class YoloDetector {
public:
    // 构造函数
    explicit YoloDetector(const DetectorConfig& config);

    // 析构函数
    ~YoloDetector() = default;

    // 禁止拷贝，只允许移动
    YoloDetector(const YoloDetector&) = delete;
    YoloDetector& operator=(const YoloDetector&) = delete;

    // 核心检测函数：输入图片，返回检测结果列表
    std::vector<Detection> detect(const cv::Mat& img);

    // 静态辅助函数：将检测结果绘制到图片上
    static void draw(cv::Mat& img, const std::vector<Detection>& objects);

private:
    cv::Mat letterbox(const cv::Mat& img, float& ratio, int& pad_x, int& pad_y);
    void nms(std::vector<Detection>& dets, float nms_thresh);
    static float iou(const Detection& a, const Detection& b);

    // 成员变量
    DetectorConfig config_;
    Ort::Env env_;
    Ort::Session session_;
    Ort::AllocatorWithDefaultOptions allocator_;

    std::string input_name_;
    std::string output_name_;
};
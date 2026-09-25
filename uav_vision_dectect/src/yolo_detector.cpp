#include "yolo_detector.hpp"

#include <iostream>
#include <algorithm>

YoloDetector::YoloDetector(const DetectorConfig& config)
    : config_(config)
    , env_(ORT_LOGGING_LEVEL_WARNING, "YoloDetector")
    , session_(nullptr)
{
    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(4);
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    try {
        OrtCUDAProviderOptions cuda_opts;
        cuda_opts.device_id = 0;
        opts.AppendExecutionProvider_CUDA(cuda_opts);
        std::cout << "YoloDetector: CUDA provider enabled" << std::endl;
    } catch (const Ort::Exception& e) {
        std::cerr << "YoloDetector: CUDA unavailable, using CPU" << std::endl;
    }

    session_ = Ort::Session(env_, config.modelPath.c_str(), opts);

    auto input_name = session_.GetInputNameAllocated(0, allocator_);
    input_name_ = input_name.get();

    auto output_name = session_.GetOutputNameAllocated(0, allocator_);
    output_name_ = output_name.get();

    std::cout << "ONNX model loaded: " << config.modelPath << std::endl;
    std::cout << "  Input:  " << input_name_ << " [1, 3, " << config.inputHeight << ", " << config.inputWidth << "]" << std::endl;
    std::cout << "  Output: " << output_name_ << std::endl;
}

cv::Mat YoloDetector::letterbox(const cv::Mat& img, float& ratio, int& pad_x, int& pad_y) {
    float r_w = (float)config_.inputWidth / img.cols;
    float r_h = (float)config_.inputHeight / img.rows;
    ratio = std::min(r_w, r_h);

    int new_w = static_cast<int>(img.cols * ratio);
    int new_h = static_cast<int>(img.rows * ratio);
    pad_x = (config_.inputWidth - new_w) / 2;
    pad_y = (config_.inputHeight - new_h) / 2;

    cv::Mat resized;
    cv::resize(img, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

    cv::Mat out(config_.inputHeight, config_.inputWidth, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(out(cv::Rect(pad_x, pad_y, new_w, new_h)));
    return out;
}

float YoloDetector::iou(const Detection& a, const Detection& b) {
    int x1 = std::max(a.box.x, b.box.x);
    int y1 = std::max(a.box.y, b.box.y);
    int x2 = std::min(a.box.x + a.box.width, b.box.x + b.box.width);
    int y2 = std::min(a.box.y + a.box.height, b.box.y + b.box.height);

    if (x1 >= x2 || y1 >= y2) return 0.0f;

    float inter_area = (x2 - x1) * (y2 - y1);
    float union_area = a.box.area() + b.box.area() - inter_area;
    return inter_area / union_area;
}

void YoloDetector::nms(std::vector<Detection>& dets, float nms_thresh) {
    std::sort(dets.begin(), dets.end(), [](const Detection& a, const Detection& b) {
        return a.conf > b.conf;
    });

    std::vector<Detection> result;
    std::vector<bool> suppressed(dets.size(), false);

    for (size_t i = 0; i < dets.size(); ++i) {
        if (suppressed[i]) continue;
        result.push_back(dets[i]);
        for (size_t j = i + 1; j < dets.size(); ++j) {
            if (!suppressed[j] && dets[i].classId == dets[j].classId &&
                iou(dets[i], dets[j]) > nms_thresh) {
                suppressed[j] = true;
            }
        }
    }
    dets = std::move(result);
}

std::vector<Detection> YoloDetector::detect(const cv::Mat& img_raw) {
    // Letterbox preprocessing
    float ratio;
    int pad_x, pad_y;
    cv::Mat input_img = letterbox(img_raw, ratio, pad_x, pad_y);

    // BGR -> RGB, HWC -> CHW, normalize to [0, 1]
    cv::Mat rgb;
    cv::cvtColor(input_img, rgb, cv::COLOR_BGR2RGB);
    rgb.convertTo(rgb, CV_32F, 1.0f / 255.0f);

    std::vector<float> input_data(3 * config_.inputHeight * config_.inputWidth);
    std::vector<cv::Mat> channels(3);
    cv::split(rgb, channels);
    for (int c = 0; c < 3; ++c) {
        memcpy(input_data.data() + c * config_.inputHeight * config_.inputWidth,
               channels[c].data, config_.inputHeight * config_.inputWidth * sizeof(float));
    }

    // Run inference
    std::array<int64_t, 4> input_shape = {1, 3, config_.inputHeight, config_.inputWidth};
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, input_data.data(), input_data.size(),
        input_shape.data(), input_shape.size());

    const char* input_names[] = {input_name_.c_str()};
    const char* output_names[] = {output_name_.c_str()};

    auto output_tensors = session_.Run(Ort::RunOptions{nullptr},
        input_names, &input_tensor, 1, output_names, 1);

    // Parse output [1, num_anchors, data_len]: cx, cy, w, h, obj_conf, cls_scores...
    auto& out_tensor = output_tensors[0];
    auto out_shape = out_tensor.GetTensorTypeAndShapeInfo().GetShape();
    int num_anchors = static_cast<int>(out_shape[1]);
    int data_len = static_cast<int>(out_shape[2]);
    const float* out_data = out_tensor.GetTensorData<float>();

    std::vector<Detection> detections;
    for (int i = 0; i < num_anchors; ++i) {
        const float* row = out_data + i * data_len;
        float obj_conf = row[4];

        if (obj_conf < config_.confThreshold) continue;

        float max_cls_score = 0;
        int class_id = 0;
        if (data_len > 5) {
            for (int c = 5; c < data_len; ++c) {
                if (row[c] > max_cls_score) {
                    max_cls_score = row[c];
                    class_id = c - 5;
                }
            }
        } else {
            max_cls_score = 1.0f;
        }

        float score = obj_conf * max_cls_score;
        if (score < config_.confThreshold) continue;

        // Map from letterbox coords back to original image coords
        float cx = (row[0] - pad_x) / ratio;
        float cy = (row[1] - pad_y) / ratio;
        float w = row[2] / ratio;
        float h = row[3] / ratio;

        Detection det;
        det.box = cv::Rect(int(cx - w/2), int(cy - h/2), int(w), int(h));
        det.conf = score;
        det.classId = class_id;
        detections.push_back(det);
    }

    nms(detections, config_.iouThreshold);
    return detections;
}

void YoloDetector::draw(cv::Mat& img, const std::vector<Detection>& objects) {
    for (const auto& obj : objects) {
        cv::rectangle(img, obj.box, cv::Scalar(255, 0, 0), 2);
        std::string label = cv::format("%.2f", obj.conf);
        cv::putText(img, label, cv::Point(obj.box.x, obj.box.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 2);
    }
}
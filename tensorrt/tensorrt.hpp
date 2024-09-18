#pragma once

#include "pch.hpp"

struct Detection {
    int classId{};
    std::string className;
    float confidence{};
    cv::Rect box;
};

class TensortRT {
    std::string mModelName;

    std::vector<std::string> classes{"bottle", "hammer"};

    InferenceWrapper mInferenceWrapper;

    auto parseModelOutput(cv::Mat& output,
                          std::vector<Detection>& detections,
                          float modelScoreThreshold = 0.75,
                          float modelNMSThreshold = 0.5) const -> void;

public:
    TensortRT();

    explicit TensortRT(std::string modelName, std::string& packagePathString);

    ~TensortRT();

    auto modelForwardPass(cv::Mat const& blob,
                          std::vector<Detection>& detections,
                          float modelScoreThreshold = 0.75,
                          float modelNMSThreshold = 0.5) const -> void;
};

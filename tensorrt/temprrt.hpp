#pragma once

#include "pch.hpp"

struct Detection {
    int classId{};
    std::string className;
    float confidence{};
    cv::Rect box;
};

class TemprRT {
    std::string mModelName;

    InferenceWrapper mInferenceWrapper;

public:
    TemprRT();

    explicit TemprRT(std::string modelName, std::string packagePathString);

    ~TemprRT();

    auto modelForwardPass(cv::Mat const& blob, cv::Mat& outputTensor) const -> void;
    
    [[nodiscard]] auto getInputTensorSize() -> std::vector<int64_t>;

	[[nodiscard]] auto getOutputTensorSize() -> std::vector<int64_t>;
};

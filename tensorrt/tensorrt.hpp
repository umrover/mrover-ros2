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

    InferenceWrapper mInferenceWrapper;

public:
    TensortRT();

    explicit TensortRT(std::string modelName, std::string packagePathString);

    ~TensortRT();

	auto modelForwardPass(cv::Mat const& inputBlobTensor, cv::Mat& outputTensor) const -> void;

	[[nodiscard]] auto getInputTensorSize() -> std::vector<int64_t>;

	[[nodiscard]] auto getOutputTensorSize() -> std::vector<int64_t>;
};

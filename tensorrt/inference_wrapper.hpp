#pragma once

#include "pch.hpp"

class Inference;

class InferenceWrapper {
    std::shared_ptr<Inference> mInference;

public:
    InferenceWrapper() = default;

    ~InferenceWrapper() = default;

    explicit InferenceWrapper(std::string onnxModelPath, std::string const& modelName, std::string const& packagePath);

    auto doDetections(cv::Mat const& img) const -> void;

    // Retrieve the output tensor from the previous forward pass
    [[nodiscard]] auto getOutputTensor() const -> cv::Mat;

	auto getInputBlobSize() -> std::vector<int64_t>;
};

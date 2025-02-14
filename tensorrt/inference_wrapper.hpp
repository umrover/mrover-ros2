#pragma once

#include "pch.hpp"

class Inference;

class InferenceWrapper {
    std::shared_ptr<Inference> mInference;

public:
    InferenceWrapper() = default;

    ~InferenceWrapper() = default;

    explicit InferenceWrapper(std::string const& modelName, std::string const& packagePath);

    auto doDetections(cv::Mat const& img) const -> void;

    // Retrieve the output tensor from the previous forward pass
    [[nodiscard]] auto getOutputTensor() const -> cv::Mat;

	[[nodiscard]] auto getInputTensorSize() -> std::vector<int64_t>;

	[[nodiscard]] auto getOutputTensorSize() -> std::vector<int64_t>;
};
#include "inference_wrapper.hpp"

#include "inference.cuh"

using namespace nvinfer1;

/**
 * cudaMemcpys CPU memory in inputTensor to GPU based on bindings
 * Queues that tensor to be passed through model
 * cudaMemcpys the result back to CPU memory
 * Requires bindings, inputTensor, stream
 * Modifies stream, outputTensor
 */

InferenceWrapper::InferenceWrapper(std::string const& modelName, std::string const& packagePath) {
	mInference = std::make_shared<Inference>(modelName, packagePath);
}

auto InferenceWrapper::doDetections(cv::Mat const& img) const -> void {
	// Execute the forward pass on the inference object
	mInference->doDetections(img);
}

auto InferenceWrapper::getOutputTensor() const -> cv::Mat {
	return mInference->getOutputTensor();
}

auto InferenceWrapper::getInputTensorSize() -> std::vector<int64_t>{
	return mInference->getInputTensorSize();
}

auto InferenceWrapper::getOutputTensorSize() -> std::vector<int64_t>{
	return mInference->getOutputTensorSize();
}
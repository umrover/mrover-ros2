#include "tensorrt.hpp"

using namespace std;

TensortRT::TensortRT() = default;

TensortRT::TensortRT(string modelName, string packagePathString) : mModelName{std::move(modelName)} {
    std::filesystem::path packagePath{std::move(packagePathString)};

    mInferenceWrapper = InferenceWrapper{mModelName, packagePath};
}

TensortRT::~TensortRT() = default;

auto TensortRT::modelForwardPass(cv::Mat const& inputBlobTensor, cv::Mat& outputTensor) const -> void {
    mInferenceWrapper.doDetections(inputBlobTensor);
    outputTensor = mInferenceWrapper.getOutputTensor();
}


auto TensortRT::getInputTensorSize() -> std::vector<int64_t>{
	return mInferenceWrapper.getInputTensorSize();
}

auto TensortRT::getOutputTensorSize() -> std::vector<int64_t>{
	return mInferenceWrapper.getOutputTensorSize();
}

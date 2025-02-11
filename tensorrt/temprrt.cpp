#include "temprrt.hpp"

using namespace std;

TemprRT::TemprRT() = default;

TemprRT::TemprRT(string modelName, string packagePathString) : mModelName{std::move(modelName)} {

    std::filesystem::path packagePath{std::move(packagePathString)};
    std::filesystem::path modelFileName = mModelName.append(".onnx");
    std::filesystem::path modelPath = packagePath / "data" / modelFileName;

    mInferenceWrapper = InferenceWrapper{modelPath, mModelName, packagePath};
}

TemprRT::~TemprRT() = default;

auto TemprRT::modelForwardPass(cv::Mat const& blob, cv::Mat& outputTensor) const -> void {
    mInferenceWrapper.doDetections(blob);
    outputTensor = mInferenceWrapper.getOutputTensor();
}

auto TemprRT::getInputTensorSize() -> std::vector<int64_t>{
	return mInferenceWrapper.getInputTensorSize();
}

auto TemprRT::getOutputTensorSize() -> std::vector<int64_t>{
	return mInferenceWrapper.getOutputTensorSize();
}
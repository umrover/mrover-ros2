#include "key_detector.hpp"
namespace mrover {
    auto KeyDetectorBase::preprocessYOLOv8Input(Model const& model, cv::Mat const& rgbImage, cv::Mat& blobSizedImage, cv::Mat& blob) -> void {
        if (model.inputTensorSize.size() != 4) {
            throw std::runtime_error("Expected Blob Size to be of size 4, are you using the correct model type?");
        }

        static constexpr double UCHAR_TO_DOUBLE = 1.0 / 255.0;

        cv::Size blobSize{static_cast<int32_t>(model.inputTensorSize[2]), static_cast<int32_t>(model.inputTensorSize[3])};
        cv::resize(rgbImage, blobSizedImage, blobSize);
        cv::dnn::blobFromImage(blobSizedImage, blob, UCHAR_TO_DOUBLE, blobSize, cv::Scalar{}, false, false);
    }

    auto KeyDetectorBase::parseYOLOv8Output(Model const& model, cv::Mat& output, std::vector<Detection>& detections) const -> void {
        std::cout << "yooo" << std::endl;
        for(int r = 0; r < output.rows; ++r){
            for(int c = 0; c < output.cols; ++c){
                std::cout << "Parse" << output.at<float>(r, c) << std::endl;
            }
        }
    }

}; // namespace mrover

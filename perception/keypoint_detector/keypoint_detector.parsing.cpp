#include "keypoint_detector.hpp"

namespace mrover {
    auto KeypointDetectorBase::preprocessYOLOv26Input(Model const& model, cv::Mat const& rgbImage, cv::Mat& blobSizedImage, cv::Mat& blob) -> void {
        if (model.inputTensorSize.size() != 4) {
            throw std::runtime_error("Expected Blob Size to be of size 4, are you using the correct model type?");
        }

        static constexpr double UCHAR_TO_DOUBLE = 1.0 / 255.0;

        cv::Size blobSize{static_cast<int32_t>(model.inputTensorSize[2]), static_cast<int32_t>(model.inputTensorSize[3])};
        cv::resize(rgbImage, blobSizedImage, blobSize);
        cv::dnn::blobFromImage(blobSizedImage, blob, UCHAR_TO_DOUBLE, blobSize, cv::Scalar{}, false, false);
    }

    auto KeypointDetectorBase::parseYOLOv26Output(Model const& model, cv::Mat& output, std::vector<Detection>& detections) const -> void {
        // The output of an end-to-end YOLO26 pose model is a 300x(6+3K) matrix
        // Each row is a detection: x1, y1, x2, y2, score, classId, then K keypoints of x, y, visibility
        // NMS is already done inside the model, so unused rows are zero padded
        static constexpr int BOX_DIMENSION = 6;
        static constexpr int KEYPOINT_DIMENSION = 3;

        if (output.cols < BOX_DIMENSION || (output.cols - BOX_DIMENSION) % KEYPOINT_DIMENSION != 0) {
            std::array<char, 100> message{};
            snprintf(message.data(), message.size(), "Something is wrong model interpretation, %d cols and %d rows", output.cols, output.rows);
            throw std::runtime_error(message.data());
        }

        int numKeypoints = (output.cols - BOX_DIMENSION) / KEYPOINT_DIMENSION;

        for (int r = 0; r < output.rows; ++r) {
            auto const* row = output.ptr<float>(r);

            // Determine if the detection is an acceptable confidence level, otherwise disregard
            float score = row[4];
            if (score <= mModelScoreThreshold) continue;

            auto classId = static_cast<int>(row[5]);
            if (classId < 0 || classId >= static_cast<int>(model.classes.size())) continue;

            // Box is given as corners in the input tensor's pixel space
            cv::Rect box{cv::Point{static_cast<int>(row[0]), static_cast<int>(row[1])},
                         cv::Point{static_cast<int>(row[2]), static_cast<int>(row[3])}};

            // Keep every keypoint in model order since the index determines what the keypoint is
            std::vector<cv::Point2f> keypoints;
            std::vector<float> keypointConfidences;
            keypoints.reserve(numKeypoints);
            keypointConfidences.reserve(numKeypoints);
            for (int k = 0; k < numKeypoints; ++k) {
                float const* keypoint = row + BOX_DIMENSION + k * KEYPOINT_DIMENSION;
                keypoints.emplace_back(keypoint[0], keypoint[1]);
                keypointConfidences.push_back(keypoint[2]);
            }

            detections.emplace_back(classId, model.classes[classId], model.colors[classId], score, box, std::move(keypoints), std::move(keypointConfidences));
        }
        
    }

}; // namespace mrover

#include "object_detector.hpp"
namespace mrover {
    auto ObjectDetectorBase::preprocessYOLOv8Input(Model const& model, cv::Mat const& rgbImage, cv::Mat& blobSizedImage, cv::Mat& blob) -> void {
        if (model.inputTensorSize.size() != 4) {
            throw std::runtime_error("Expected Blob Size to be of size 4, are you using the correct model type?");
        }

        static constexpr double UCHAR_TO_DOUBLE = 1.0 / 255.0;

        cv::Size blobSize{static_cast<int32_t>(model.inputTensorSize[2]), static_cast<int32_t>(model.inputTensorSize[3])};
        cv::resize(rgbImage, blobSizedImage, blobSize);
        cv::dnn::blobFromImage(blobSizedImage, blob, UCHAR_TO_DOUBLE, blobSize, cv::Scalar{}, false, false);
    }

    auto ObjectDetectorBase::parseYOLOv8Output(Model const& model, cv::Mat& output, std::vector<Detection>& detections) const -> void {
        // Parse model specific dimensioning from the output

        // The input to this function is expecting a YOLOv8 style model, thus the dimensions should be > rows
        if (output.cols <= output.rows) {
            std::array<char, 75> message{};
            snprintf(message.data(), message.size(), "Something is wrong model interpretation, %d cols and %d rows", output.cols, output.rows);
            throw std::runtime_error(message.data());
        }

        int numCandidates = output.cols;
        int predictionDimension = output.rows;

        // The output of the model is a batchSizex84x8400 matrix
        // This converts the model to a TODO: Check this dimensioning
        output = output.reshape(1, predictionDimension);
        cv::transpose(output, output);

        // This function expects the image to already be in the correct format thus no distrotion is needed
        float const xFactor = 1.0;
        float const yFactor = 1.0;

        // Intermediate Storage Containers
        std::vector<int> classIds;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;

        // Each row of the output is a detection with a bounding box and associated class scores
        for (int r = 0; r < numCandidates; ++r) {
            // Skip first four values as they are the box data
            cv::Mat scores = output.row(r).colRange(4, predictionDimension);

            cv::Point classId;
            double maxClassScore;
            cv::minMaxLoc(scores, nullptr, &maxClassScore, nullptr, &classId);

            // Determine if the class is an acceptable confidence level, otherwise disregard
            if (maxClassScore <= mModelScoreThreshold) continue;

            confidences.push_back(static_cast<float>(maxClassScore));
            classIds.push_back(classId.x);

            // Get the bounding box data
            cv::Mat box = output.row(r).colRange(0, 4);
            auto x = box.at<float>(0);
            auto y = box.at<float>(1);
            auto w = box.at<float>(2);
            auto h = box.at<float>(3);

            // Cast the corners into integers to be used on pixels
            auto left = static_cast<int>((x - 0.5 * w) * xFactor);
            auto top = static_cast<int>((y - 0.5 * h) * yFactor);
            auto width = static_cast<int>(w * xFactor);
            auto height = static_cast<int>(h * yFactor);

            // Push abck the box into storage
            boxes.emplace_back(left, top, width, height);
        }

        //Coalesce the boxes into a smaller number of distinct boxes
        std::vector<int> nmsResult;
        cv::dnn::NMSBoxes(boxes, confidences, mModelScoreThreshold, mModelNMSThreshold, nmsResult);

        // Fill in the output Detections Vector
        for (int i: nmsResult) {
            //Push back the detection into the for storage vector
            detections.emplace_back(classIds[i], model.classes[classIds[i]], confidences[i], boxes[i]);
        }
    }

}; // namespace mrover

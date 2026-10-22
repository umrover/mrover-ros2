#include "keypoint_detector.hpp"

namespace mrover {

    auto KeypointDetectorBase::toggleMode(mrover::srv::ToggleObjectDetector::Request::ConstSharedPtr& request, mrover::srv::ToggleObjectDetector::Response::SharedPtr& response) -> void {
        auto setMode = request->waypoint.val;
        switch (setMode) {
            case msg::WaypointType::NO_SEARCH:
                currentModel = nullptr;
                currentTensorRT = nullptr;
                break;
            case msg::WaypointType::POST:
                currentModel = nullptr;
                currentTensorRT = nullptr;
                break;
            case msg::WaypointType::KEY_POINT:
                currentModel = &mKeypointModel;
                currentTensorRT = &mKeypointTensorRT;
                mModelScoreThreshold = mKeypointScoreThreshold;
                break;
            default:
                response->success = false;
                RCLCPP_INFO_STREAM(get_logger(), "Received malformed service call");
                return;
        }
        response->success = true;
        RCLCPP_INFO_STREAM(get_logger(), std::format("Set keypoint detector to {}", setMode));
        RCLCPP_INFO_STREAM(this->get_logger(), std::format("Score Threshold is Now {}", mModelScoreThreshold));
    }

    auto KeypointDetectorBase::drawDetectionBoxes(cv::InputOutputArray image, std::span<Detection const> detections) -> void {
        // Draw the detected object's bounding boxes and keypoints on the image for each of the objects detected
        for (std::size_t i = 0; i < detections.size(); i++) {
            // Font color will change for each different detection
            cv::rectangle(image, detections[i].box, detections[i].fontColor, 1, cv::LINE_8, 0);

            // Draw each keypoint as a filled circle
            for (auto const& keypoint: detections[i].keypoints) {
                cv::circle(image, keypoint, 3, detections[i].fontColor, cv::FILLED);
            }

            // Put the text on the image
            cv::Point textPosition(80, static_cast<int>(80 * (i + 1)));
            constexpr int fontSize = 1;
            constexpr int fontWeight = 2;
            std::string text = detections[i].className + " " + std::to_string(detections[i].confidence);
            putText(image, text, textPosition, cv::FONT_HERSHEY_COMPLEX, fontSize, detections[i].fontColor, fontWeight); // Putting the text in the matrix
        }
    }

    auto KeypointDetectorBase::publishDetectedObjects(cv::InputArray const& image) -> void {
        mDetectionsImageMessage.header.stamp = get_clock()->now();
        mDetectionsImageMessage.height = image.rows();
        mDetectionsImageMessage.width = image.cols();
        mDetectionsImageMessage.encoding = sensor_msgs::image_encodings::BGRA8;
        mDetectionsImageMessage.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        mDetectionsImageMessage.step = 4 * mDetectionsImageMessage.width;
        mDetectionsImageMessage.data.resize(mDetectionsImageMessage.step * mDetectionsImageMessage.height);
        cv::Mat debugImageWrapper{image.size(), CV_8UC4, mDetectionsImageMessage.data.data()};
        cv::cvtColor(image, debugImageWrapper, cv::COLOR_RGB2BGRA);

        mDebugImgPub->publish(mDetectionsImageMessage);
    }

    auto ImageKeypointDetector::imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);
        assert(msg->encoding == sensor_msgs::image_encodings::BGRA8);

        mLoopProfiler.beginLoop();

        cv::Mat bgraImage{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, const_cast<uint8_t*>(msg->data.data())};
        cv::cvtColor(bgraImage, mRgbImage, cv::COLOR_BGRA2RGB);

        // Convert the RGB Image into the blob Image format

        if (currentModel && currentTensorRT) {
            cv::Mat blobSizedImage;
            currentModel->rgbImageToBlob(*currentModel, mRgbImage, blobSizedImage, mImageBlob);

            mLoopProfiler.measureEvent("Conversion");
        
            // Run the blob through the model
            std::vector<Detection> detections{};
            cv::Mat outputTensor;
            currentTensorRT->modelForwardPass(mImageBlob, outputTensor);

            currentModel->outputTensorToDetections(*currentModel, outputTensor, detections);

            mLoopProfiler.measureEvent("Execution");

            mrover::msg::ImageTargets targets{};
            for (auto const& detection: detections) {
                mrover::msg::ImageTarget target;
                target.name = detection.className;
                target.bearing = getObjectBearing(blobSizedImage, detection.box);
                targets.targets.emplace_back(target);
            }

            mTargetsPub->publish(targets);

            resizeBoundingBoxes(mRgbImage.size(), detections);
            drawDetectionBoxes(mRgbImage, detections);

            mLoopProfiler.measureEvent("Publication");
        }

        if (mDebug) {
            publishDetectedObjects(mRgbImage);
        }
    }

    auto ImageKeypointDetector::getObjectBearing(cv::InputArray const& image, cv::Rect const& box) const -> float {
        cv::Point2f center = cv::Point2f{box.tl()} + cv::Point2f{box.size()} / 2;
        float xNormalized = center.x / static_cast<float>(image.cols());
        float xRecentered = 0.5f - xNormalized;
        float bearingDegrees = xRecentered * mCameraHorizontalFov;
        return bearingDegrees * std::numbers::pi_v<float> / 180.0f;
    }

    auto KeypointDetectorBase::resizeBoundingBoxes(cv::Size const& outputSpace, std::vector<Detection>& detections) const -> void {
        float xRatio = static_cast<float>(outputSpace.width) / static_cast<float>(currentModel->inputTensorSize[2]);
        float yRatio = static_cast<float>(outputSpace.height) / static_cast<float>(currentModel->inputTensorSize[3]);

        for (auto& det: detections) {
            cv::Rect newBoundingBox;
            newBoundingBox.x = static_cast<int>(static_cast<float>(det.box.x) * xRatio);
            newBoundingBox.y = static_cast<int>(static_cast<float>(det.box.y) * yRatio);
            newBoundingBox.width = static_cast<int>(static_cast<float>(det.box.width) * xRatio);
            newBoundingBox.height = static_cast<int>(static_cast<float>(det.box.height) * yRatio);
            std::swap(newBoundingBox, det.box);

            for (auto& keypoint: det.keypoints) {
                keypoint.x *= xRatio;
                keypoint.y *= yRatio;
            }
        }
    }

} // namespace mrover

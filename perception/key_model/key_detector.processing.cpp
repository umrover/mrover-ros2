#include "key_detector.hpp"

namespace mrover {
    auto KeyDetectorBase::spiralSearchForValidPoint(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& cloudPtr, std::size_t u, std::size_t v, std::size_t width, std::size_t height) const -> std::optional<SE3d> {
        // See: https://stackoverflow.com/a/398302
        auto xc = static_cast<int>(u), yc = static_cast<int>(v);
        auto sw = static_cast<int>(width), sh = static_cast<int>(height);
        auto ih = static_cast<int>(cloudPtr->height), iw = static_cast<int>(cloudPtr->width);
        int sx = 0, sy = 0; // Spiral coordinates starting at (0, 0)
        int dx = 0, dy = -1;
        std::size_t bigger = std::max(width, height);
        std::size_t maxIterations = bigger * bigger;
        for (std::size_t i = 0; i < maxIterations; i++) {
            if (-sw / 2 < sx && sx <= sw / 2 && -sh / 2 < sy && sy <= sh / 2) {
                int ix = xc + sx, iy = yc + sy; // Image coordinates

                if (ix < 0 || ix >= iw || iy < 0 || iy >= ih) {
                    RCLCPP_INFO_STREAM(get_logger(), std::format("Spiral query is outside the image: [{}, {}]", ix, iy));
                    continue;
                }

                Point const& point = reinterpret_cast<Point const*>(cloudPtr->data.data())[ix + iy * cloudPtr->width];
                if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;

                return std::make_optional<SE3d>(R3d{point.x, point.y, point.z}, SO3d::Identity());
            }

            if (sx == sy || (sx < 0 && sx == -sy) || (sx > 0 && sx == 1 - sy)) {
                dy = -dy;
                std::swap(dx, dy);
            }

            sx += dx;
            sy += dy;
        }
        return std::nullopt;
    }

    auto KeyDetectorBase::drawDetectionBoxes(cv::InputOutputArray image, std::span<Detection const> detections) const -> void {
        // Draw the detected object's bounding boxes on the image for each of the objects detected
        std::array const fontColors{cv::Scalar{0, 4, 227}, cv::Scalar{232, 115, 5}};

        double const scaleFactorX{static_cast<double>(image.cols()) / static_cast<double>(mKeyDetectionModel.inputTensorSize[2])};
        double const scaleFactorY{static_cast<double>(image.rows()) / static_cast<double>(mKeyDetectionModel.inputTensorSize[3])};

        for (std::size_t i = 0; i < detections.size(); i++) {
            // Font color will change for each different detection
            cv::Scalar const& fontColor = fontColors.at(detections[i].classId);
            cv::Rect box(static_cast<int>(detections[i].box.x * scaleFactorX), static_cast<int>(detections[i].box.y * scaleFactorY), static_cast<int>(detections[i].box.width * scaleFactorX), static_cast<int>(detections[i].box.height * scaleFactorY));
            cv::rectangle(image, box, fontColor, 1, cv::LINE_8, 0);

            // Put the text on the image
            cv::Point textPosition(80, static_cast<int>(80 * (i + 1)));
            constexpr int fontSize = 1;
            constexpr int fontWeight = 2;
            putText(image, detections[i].className, textPosition, cv::FONT_HERSHEY_COMPLEX, fontSize, fontColor, fontWeight); // Putting the text in the matrix
        }
    }

    auto KeyDetectorBase::publishDetectedObjects(cv::InputArray const& image) -> void {
        mDetectionsImageMessage.header.stamp = get_clock()->now();
        mDetectionsImageMessage.height = image.rows();
        mDetectionsImageMessage.width = image.cols();
        mDetectionsImageMessage.encoding = sensor_msgs::image_encodings::BGRA8;
        mDetectionsImageMessage.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        mDetectionsImageMessage.step = 4 * mDetectionsImageMessage.width;
        mDetectionsImageMessage.data.resize(mDetectionsImageMessage.step * mDetectionsImageMessage.height);
        std::memcpy(mDetectionsImageMessage.data.data(), image.getMat().data, mDetectionsImageMessage.data.size());

        mDebugImgPub->publish(mDetectionsImageMessage);
    }

    auto ImageKeyDetector::imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);
        assert(msg->encoding == sensor_msgs::image_encodings::BGRA8);

        mLoopProfiler.beginLoop();

        cv::Mat bgraImage{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, const_cast<uint8_t*>(msg->data.data())};

        cv::Mat temp;
<<<<<<< HEAD
        temp = cv::imread("/home/john/ros2_ws/src/mrover/perception/key_detector/keyrover/../datasets/test/image/f91.png", cv::IMREAD_COLOR);
=======

        std::filesystem::path packagePath = std::filesystem::path{ament_index_cpp::get_package_prefix("mrover")} / ".." / ".." / "src" / "mrover" / "perception" / "key_detector" / "datasets" / "test" / "image" / "f91.png" ;

        temp = cv::imread(packagePath.c_str(), cv::IMREAD_COLOR);
>>>>>>> 95f5bce (made file path not depend on machine)

        cv::cvtColor(temp, bgraImage, cv::COLOR_BGR2BGRA);

        // Convert the RGB Image into the blob Image format
        mKeyDetectionModel.preprocess(mKeyDetectionModel, bgraImage, mImageBlob);

        mTextCoordModel.preprocess(mTextCoordModel, bgraImage, mTextCoordsBlob);

        mLoopProfiler.measureEvent("Conversion");

        // Run the blob through the model
        std::vector<Detection> detections{};
        cv::Mat outputTensor;
        mKeyDetectionTensorRT.modelForwardPass(mImageBlob, outputTensor);

        mKeyDetectionModel.postprocess(mKeyDetectionModel, outputTensor);

        parseYOLOv8Output(mKeyDetectionModel, outputTensor, detections);

        // Text Coords Inference
        mTextCoordsTensorRT.modelForwardPass(mTextCoordsBlob, outputTensor);

        mTextCoordModel.postprocess(mTextCoordModel, outputTensor);

        matchKeyDetections(outputTensor, detections);

        publishDetectedObjects(outputTensor);

        mLoopProfiler.measureEvent("Execution");

        mrover::msg::ImageTargets targets{};
        for (auto const& [classId, className, confidence, box]: detections) {
            mrover::msg::ImageTarget target;
            target.name = className;
            target.bearing = getObjectBearing(mKeyDetectionModel.inputTensorSize[2], box);
            targets.targets.emplace_back(target);
        }

        mTargetsPub->publish(targets);

        drawDetectionBoxes(bgraImage, detections);
        if (mDebug) {
            //publishDetectedObjects(bgraImage);
        }

        mLoopProfiler.measureEvent("Publication");
    }

    auto ImageKeyDetector::getObjectBearing(std::size_t imageWidth, cv::Rect const& box) const -> float {
        cv::Point2f center = cv::Point2f{box.tl()} + cv::Point2f{box.size()} / 2;
        float xNormalized = center.x / static_cast<float>(imageWidth);
        float xRecentered = 0.5f - xNormalized;
        float bearingDegrees = xRecentered * mCameraHorizontalFov;
        return bearingDegrees * std::numbers::pi_v<float> / 180.0f;
    }

} // namespace mrover

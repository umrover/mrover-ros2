#include "object_detector.hpp"

namespace mrover {

    auto StereoObjectDetector::pointCloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr const& msg) -> void {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        mLoopProfiler.beginLoop();

        // Adjust the picture size to be in line with the expected img size from the Point Cloud
        if (static_cast<int>(msg->height) != mRgbImage.rows || static_cast<int>(msg->width) != mRgbImage.cols) {
            RCLCPP_INFO_STREAM(get_logger(), std::format("Image size changed from [{}, {}] to [{}, {}]", mRgbImage.cols, mRgbImage.rows, msg->width, msg->height));
            mRgbImage = cv::Mat{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3, cv::Scalar{0, 0, 0, 0}};
        }

        // if 0x0 image
        if (!mRgbImage.total()) {
            return;
        }

        convertPointCloudToRGB(msg, mRgbImage);

        // Convert the RGB Image into the blob Image format
        cv::Mat blobSizedImage;
		mModel.rbgImageToBlob(mModel, mRgbImage, blobSizedImage);

        mLoopProfiler.measureEvent("Conversion");

        // Run the blob through the model
        std::vector<Detection> detections{};
        cv::Mat outputTensor;
        mTensorRT.modelForwardPass(mImageBlob, outputTensor);
		
		mModel.outputTensorToDetections(mModel, outputTensor, detections);

        mLoopProfiler.measureEvent("Execution");

        // Increment Object hit counts if theyre seen
        // Decrement Object hit counts if they're not seen
        updateHitsObject(msg, detections);

        // Draw the bounding boxes on the image
        drawDetectionBoxes(blobSizedImage, detections);
        publishDetectedObjects(blobSizedImage);

        mLoopProfiler.measureEvent("Publication");
    }

    auto ObjectDetectorBase::updateHitsObject(sensor_msgs::msg::PointCloud2::UniquePtr const& msg, std::span<Detection const> detections, cv::Size const& imageSize) -> void {
        // Set of flags indicating if the given object has been seen
        // TODO(quintin): Do not hard code exactly two classes
        std::bitset<2> seenObjects{0b00};
        for (auto const& [classId, className, confidence, box] : detections) {
            // Resize from blob space to image space
            cv::Point2f centerInBlob = cv::Point2f{box.tl()} + cv::Point2f{box.size()} / 2;
            float xRatio = static_cast<float>(msg->width) / static_cast<float>(imageSize.width);
            float yRatio = static_cast<float>(msg->height) / static_cast<float>(imageSize.height);
            std::size_t centerXInImage = std::lround(centerInBlob.x * xRatio);
            std::size_t centerYInImage = std::lround(centerInBlob.y * yRatio);

            assert(static_cast<std::size_t>(classId) < mObjectHitCounts.size());

            if (seenObjects[classId]) return;

            seenObjects[classId] = true;

            // Get the object's position in 3D from the point cloud and run this statement if the optional has a value
            if (std::optional<SE3d> objectInCamera = spiralSearchForValidPoint(msg, centerXInImage, centerYInImage, box.width, box.height)) {
                try {
                    std::string objectImmediateFrame = std::format("immediate{}", className);
                    // Push the immediate detections to the camera frame
                    SE3Conversions::pushToTfTree(*mTfBroadcaster, objectImmediateFrame, mCameraFrame, objectInCamera.value(), get_clock()->now());
                    // Since the object is seen we need to increment the hit counter
                    mModel.objectHitCounts[classId] = std::min(mObjMaxHitcount, mModel.objectHitCounts[classId] + mObjIncrementWeight);

                    // Only publish to permament if we are confident in the object
                    if (mModel.objectHitCounts[classId] > mObjHitThreshold) {
                        std::string objectPermanentFrame = className;
                        // Grab the object inside of the camera frame and push it into the map frame
                        SE3d objectInMap = SE3Conversions::fromTfTree(*mTfBuffer, objectImmediateFrame, mWorldFrame);
                        SE3Conversions::pushToTfTree(*mTfBroadcaster, objectPermanentFrame, mWorldFrame, objectInMap, get_clock()->now());
                    }

                } catch (tf2::ExtrapolationException const&) {
                    RCLCPP_INFO_STREAM(get_logger(), "Old data for immediate objects");
                } catch (tf2::LookupException const&) {
                    RCLCPP_INFO_STREAM(get_logger(), "Expected transform for immediate objects");
                } catch (tf2::ConnectivityException const&) {
                    RCLCPP_INFO_STREAM(get_logger(), "Expected connection to odom frame. Is visual odometry running?");
                }
            }
        }

        for (std::size_t i = 0; i < seenObjects.size(); i++) {
            if (seenObjects[i]) continue;

            assert(i < mObjectHitCounts.size());
            mModel.objectHitCounts[i] = std::max(0, mModel.objectHitCounts[i] - mObjDecrementWeight);
        }
    }

    auto ObjectDetectorBase::spiralSearchForValidPoint(sensor_msgs::msg::PointCloud2::UniquePtr const& cloudPtr, std::size_t u, std::size_t v, std::size_t width, std::size_t height) const -> std::optional<SE3d> {
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

    auto ObjectDetectorBase::drawDetectionBoxes(cv::InputOutputArray image, std::span<Detection const> detections) -> void {
        // Draw the detected object's bounding boxes on the image for each of the objects detected
        std::array const fontColors{cv::Scalar{0, 4, 227}, cv::Scalar{232, 115, 5}};
        for (std::size_t i = 0; i < detections.size(); i++) {
            // Font color will change for each different detection
            cv::Scalar const& fontColor = fontColors.at(detections[i].classId);
            cv::rectangle(image, detections[i].box, fontColor, 1, cv::LINE_8, 0);

            // Put the text on the image
            cv::Point textPosition(80, static_cast<int>(80 * (i + 1)));
            constexpr int fontSize = 1;
            constexpr int fontWeight = 2;
            putText(image, detections[i].className, textPosition, cv::FONT_HERSHEY_COMPLEX, fontSize, fontColor, fontWeight); // Putting the text in the matrix
        }
    }

    auto ObjectDetectorBase::publishDetectedObjects(cv::InputArray image) -> void {
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

    auto StereoObjectDetector::convertPointCloudToRGB(sensor_msgs::msg::PointCloud2::UniquePtr const& msg, cv::Mat const& image) -> void {
        auto* pixelPtr = reinterpret_cast<cv::Vec3b*>(image.data);
        auto* pointPtr = reinterpret_cast<Point const*>(msg->data.data());
        std::for_each(std::execution::par_unseq, pixelPtr, pixelPtr + image.total(), [&](cv::Vec3b& pixel) {
            std::size_t const i = &pixel - pixelPtr;
            pixel[0] = pointPtr[i].r;
            pixel[1] = pointPtr[i].g;
            pixel[2] = pointPtr[i].b;
        });
    }

} // namespace mrover

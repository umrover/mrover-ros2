#include "tag_detector.hpp"

#include "point.hpp"

namespace mrover {

    /**
     * For each tag we have detected so far, fuse point cloud information.
     * This information is where it is in the world.
     *
     * @param msg   Point cloud message
     */
    auto StereoTagDetector::pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) -> void {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        // OpenCV needs a dense BGR image |BGR|...| but out point cloud is
        // |BGRAXYZ...|...| So we need to copy the data into the correct format
        if (static_cast<int>(msg->height) != mBgrImage.rows || static_cast<int>(msg->width) != mBgrImage.cols) {
            RCLCPP_INFO_STREAM(get_logger(), std::format("Image size changed from [{}, {}] to [{}, {}]", mBgrImage.cols, mBgrImage.rows, msg->width, msg->height));
            mBgrImage = cv::Mat{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3, cv::Scalar{0, 0, 0}};
        }
        auto* pixelPtr = reinterpret_cast<cv::Vec3b*>(mBgrImage.data);
        auto* pointPtr = reinterpret_cast<Point const*>(msg->data.data());
        std::for_each(std::execution::par_unseq, pixelPtr, pixelPtr + mBgrImage.total(), [&](cv::Vec3b& pixel) {
            std::size_t i = &pixel - pixelPtr;
            pixel[0] = pointPtr[i].b;
            pixel[1] = pointPtr[i].g;
            pixel[2] = pointPtr[i].r;
        });
        mProfiler.measureEvent("Conversion");

        // Detect the tag vertices in screen space and their respective ids
        // {mImmediateCorneres, mImmediateIds} are the outputs from OpenCV
        cv::aruco::detectMarkers(mBgrImage, mDictionary, mImmediateCorners, mImmediateIds, mDetectorParams);
        mProfiler.measureEvent("Detection");

        publishThresholdedImage();
        mProfiler.measureEvent("Threshold");

        // Update ID, image center, and increment hit count for all detected tags
        for (std::size_t i = 0; i < mImmediateIds.size(); ++i) {
            int id = mImmediateIds[i];
            Tag& tag = mTags[id];
            tag.hitCount = std::clamp(tag.hitCount + mTagIncrementWeight, 0, mMaxTagHitCount);
            tag.hitCount = std::clamp(tag.hitCount + mTagIncrementWeight, 0, mMaxTagHitCount);
            tag.id = id;
            tag.imageCenter = std::reduce(mImmediateCorners[i].begin(), mImmediateCorners[i].end()) / static_cast<float>(mImmediateCorners[i].size());
            auto approximateSize = static_cast<std::size_t>(std::sqrt(cv::contourArea(mImmediateCorners[i])));
            tag.tagInCam = spiralSearchForValidPoint(msg, std::lround(tag.imageCenter.x), std::lround(tag.imageCenter.y), approximateSize, approximateSize);

            if (!tag.tagInCam) continue;

            // Publish tag to immediate
            std::string immediateFrameId = std::format("immediateTag{}", tag.id);
            SE3Conversions::pushToTfTree(mTfBroadcaster, immediateFrameId, mCameraFrameId, tag.tagInCam.value(), get_clock()->now());
        }
        // Handle tags that were not seen this update
        // Decrement their hit count and remove if they hit zero
        auto it = mTags.begin();
        while (it != mTags.end()) {
            if (auto& [id, tag] = *it; std::ranges::find(mImmediateIds, id) == mImmediateIds.end()) {
                tag.hitCount -= mTagDecrementWeight;
                tag.tagInCam = std::nullopt;
                if (tag.hitCount <= 0) {
                    it = mTags.erase(it);
                    continue;
                }
            }
            ++it;
        }
        // Publish all tags to the tf tree that have been seen enough times
        for (auto const& [id, tag]: mTags) {
            if (tag.hitCount >= mMinTagHitCountBeforePublish && tag.tagInCam) {
                try {
                    // Use the TF tree to transform the tag from the camera frame to the map frame
                    // Then publish it in the map frame persistently
                    std::string immediateFrameId = std::format("immediateTag{}", tag.id);
                    SE3d tagInParent = SE3Conversions::fromTfTree(mTfBuffer, immediateFrameId, mMapFrameId);
                    SE3Conversions::pushToTfTree(mTfBroadcaster, std::format("tag{}", tag.id), mMapFrameId, tagInParent, get_clock()->now());
                } catch (tf2::ExtrapolationException const&) {
                    RCLCPP_WARN_STREAM(get_logger(), "Old data for immediate tag");
                } catch (tf2::LookupException const&) {
                    RCLCPP_WARN_STREAM(get_logger(), "Expected transform for immediate tag");
                } catch (tf2::ConnectivityException const&) {
                    RCLCPP_WARN_STREAM(get_logger(), "Expected connection to odom frame. Is visual odometry running?");
                }
            }
        }
        mProfiler.measureEvent("Filtration");

        publishDetectedTags();
        mProfiler.measureEvent("Publication");
    }

    auto ImageTagDetector::imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);
        assert(msg->encoding == sensor_msgs::image_encodings::BGRA8);

        cv::Mat bgraImage{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, const_cast<std::uint8_t*>(msg->data.data())}; // No copy is made, it simply wraps the pointer
        cv::cvtColor(bgraImage, mBgrImage, cv::COLOR_BGRA2BGR);
        mProfiler.measureEvent("Conversion");

        // Detect the tag vertices in screen space and their respective ids
        // {mImmediateCorneres, mImmediateIds} are the outputs from OpenCV
        cv::aruco::detectMarkers(mBgrImage, mDictionary, mImmediateCorners, mImmediateIds, mDetectorParams);
        mProfiler.measureEvent("Detection");

        msg::ImageTargets targets;
        for (std::size_t i = 0; i < mImmediateIds.size(); ++i) {
            msg::ImageTarget newTarget;
            newTarget.name = std::format("tag{}", mImmediateIds[i]);
            newTarget.bearing = getTagBearing(mBgrImage, mImmediateCorners[i]);
            targets.targets.push_back(newTarget);
        }
        mTargetsPub->publish(targets);
        publishDetectedTags();
        mProfiler.measureEvent("Publication");
    }

    auto TagDetectorBase::publishDetectedTags() -> void {
        if (mDetectedImagePub->get_subscription_count()) {
            cv::aruco::drawDetectedMarkers(mBgrImage, mImmediateCorners, mImmediateIds);
            // Max number of tags the hit counter can display = 10;
            if (!mTags.empty()) {
                // TODO: remove some magic numbers in this block
                int tagCount = 1;
                auto tagBoxWidth = static_cast<int>(mBgrImage.cols / (mTags.size() * 2));
                for (auto& [id, tag]: mTags) {
                    cv::Scalar color{255, 0, 0};
                    cv::Point pt{tagBoxWidth * tagCount, mBgrImage.rows / 10};
                    std::string text = std::format("id{}:{}", id, tag.hitCount);
                    cv::putText(mBgrImage, text, pt, cv::FONT_HERSHEY_COMPLEX, mBgrImage.cols / 800.0, color, mBgrImage.cols / 300);
                    ++tagCount;
                }
            }
            mDetectionsImageMessage.header.stamp = get_clock()->now();
            mDetectionsImageMessage.header.frame_id = mCameraFrameId;
            mDetectionsImageMessage.height = mBgrImage.rows;
            mDetectionsImageMessage.width = mBgrImage.cols;
            mDetectionsImageMessage.encoding = sensor_msgs::image_encodings::BGRA8;
            mDetectionsImageMessage.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
            mDetectionsImageMessage.step = 4 * mDetectionsImageMessage.width;
            mDetectionsImageMessage.data.resize(mDetectionsImageMessage.step * mDetectionsImageMessage.height);
            cv::Mat imageMessageWrapper{mBgrImage.rows, mBgrImage.cols, CV_8UC4, mDetectionsImageMessage.data.data()};
            cv::cvtColor(mBgrImage, imageMessageWrapper, cv::COLOR_BGR2BGRA);

            mDetectedImagePub->publish(mDetectionsImageMessage);
        }

        if (std::size_t detectedCount = mImmediateIds.size(); !mPrevDetectedCount.has_value() || detectedCount != mPrevDetectedCount.value()) {
            // Note(quintin): Replace this with "formatting ranges" when it is available
            std::ostringstream messageStream;
            messageStream << std::format("Detected {} immediate tags: ", detectedCount);
            messageStream << "{ ";
            std::ranges::copy(mImmediateIds, std::ostream_iterator<int>{messageStream, " "});
            messageStream << "}";
            RCLCPP_INFO_STREAM(get_logger(), messageStream.str());
            mPrevDetectedCount = detectedCount;
        }
    }

    auto StereoTagDetector::spiralSearchForValidPoint(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& cloudPtr,
                                                      std::size_t u, std::size_t v,
                                                      std::size_t width, std::size_t height) const -> std::optional<SE3d> {
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
                    RCLCPP_WARN_STREAM(get_logger(), std::format("Spiral query is outside the image: [{}, {}]", ix, iy));
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

    auto ImageTagDetector::getTagBearing(cv::InputArray image, std::span<cv::Point2f const> tagCorners) const -> float {
        // Takes the average of the corners
        cv::Point2f center = std::reduce(tagCorners.begin(), tagCorners.end()) / static_cast<float>(tagCorners.size());
        float xNormalized = center.x / static_cast<float>(image.cols());
        float xRecentered = 0.5f - xNormalized;
        float bearingDegrees = xRecentered * mCameraHorizontalFOV;
        return bearingDegrees * std::numbers::pi_v<float> / 180.0f;
    }

} // namespace mrover

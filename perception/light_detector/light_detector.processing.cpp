#include "light_detector.hpp"
#include <cfloat>
#include <cmath>
#include <execution>
#include <manif/impl/se3/SE3.h>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <rclcpp/logging.hpp>
#include <string>

namespace mrover {
    auto LightDetector::convertPointCloudToRGB(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg, cv::Mat const& image) -> void {
        auto* pixelPtr = reinterpret_cast<cv::Vec3b*>(image.data);
        auto* pointPtr = reinterpret_cast<Point const*>(msg->data.data());
        std::for_each(std::execution::par_unseq, pixelPtr, pixelPtr + image.total(), [&](cv::Vec3b& pixel) {
            std::size_t const i = &pixel - pixelPtr;
            pixel[0] = pointPtr[i].r;
            pixel[1] = pointPtr[i].g;
            pixel[2] = pointPtr[i].b;
        });
    }

    auto LightDetector::round_to(double value, double precision) -> double{
        double val = std::round(value / precision) * precision;
        return (floor((val*2)+0.5)/2);
    }

    auto LightDetector::getPointFromPointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& cloudPtr, std::pair<int, int> coordinates) -> std::optional<SE3d>{
        Point const& p = reinterpret_cast<Point const*>(cloudPtr->data.data())[coordinates.second + coordinates.first * cloudPtr->width];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)){
            //RCLCPP_INFO_STREAM(get_logger(),"failed");
            return std::nullopt;
        }
        RCLCPP_INFO_STREAM(get_logger(),"x: " << p.x << " y: " << p.y << " z: " << p.z);
        return std::make_optional<SE3d>(R3d{p.x, p.y, p.z}, SO3d::Identity());
    }
    
    auto LightDetector::imageCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) -> void {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);
        assert(msg->encoding == sensor_msgs::image_encodings::BGRA8);

        if (static_cast<int>(msg->height) != mImgRGB.rows || static_cast<int>(msg->width) != mImgRGB.cols) {
            RCLCPP_INFO_STREAM(get_logger(), std::format("Image size changed from [{}, {}] to [{}, {}]", mImgRGB.cols, mImgRGB.rows, msg->width, msg->height));
            mImgRGB = cv::Mat{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3, cv::Scalar{0, 0, 0, 0}};
        }

        if (!mImgRGB.total()) {
            return;
        }
        convertPointCloudToRGB(msg, mImgRGB);
        
        //Convert to blob
        cv::Mat blobSizedImage;
        mModel.rgbImageToBlob(mModel, mImgRGB, blobSizedImage, mImageBlob);
        RCLCPP_INFO_STREAM(get_logger(),"BLOB DIMENSIONS" << blobSizedImage.cols << "," << blobSizedImage.rows);

		std::vector<Detection> detections{};
        cv::Mat outputTensor;
        mTensorRT.modelForwardPass(mImageBlob, outputTensor);
        mModel.outputTensorToDetections(mModel, outputTensor, detections);
        std::vector<std::pair<int, int>> centroids = convert(detections);
        drawDetectionBoxes(blobSizedImage, detections, centroids);
        //convert centroids to se3 here and put into unordered_map
        for(auto centroid : centroids){
            std::optional<SE3d> lightInCamera = getPointFromPointCloud(msg, centroid);

        }
        //caching(detections);
        if (mDebug) {
            publishDebugObjects(blobSizedImage);
        }
    }

    auto LightDetector::drawDetectionBoxes(cv::InputOutputArray image, std::span<Detection const> detections, std::vector<std::pair<int, int>> &centroids) -> void {
        // Draw the detected object's bounding boxes on the image for each of the objects detected
        std::array const fontColors{cv::Scalar{0, 4, 227}, cv::Scalar{232, 115, 5}};
        for (std::size_t i = 0; i < detections.size(); i++) {
            // Font color will change for each different detection
            cv::Scalar const& fontColor = fontColors.at(detections[i].classId);
            cv::rectangle(image, detections[i].box, fontColor, 1, cv::LINE_8, 0);
            RCLCPP_INFO_STREAM(get_logger(), "TOP LEFT X: " << detections[i].box.x << " TOP LEFT Y: " << detections[i].box.y << " WIDTH: " << detections[i].box.width << " HEIGHT: " << detections[i].box.height);
            // Put the text on the image
            cv::Point textPosition(80, static_cast<int>(80 * (i + 1)));
            constexpr int fontSize = 1;
            constexpr int fontWeight = 2;
            putText(image, detections[i].className + "X: " + std::to_string(centroids[i].first) + "Y: " + std::to_string(centroids[i].second) , textPosition, cv::FONT_HERSHEY_COMPLEX, fontSize, fontColor, fontWeight); // Putting the text in the matrix
        }
    }

    auto LightDetector::convert(const std::vector<Detection> &detections) -> std::vector<std::pair<int, int>>{
        std::vector<std::pair<int, int>> centroids;
        for(auto &detection : detections){
            int height = detection.box.height;
            int width = detection.box.width;
            int top_left_x = detection.box.x;
            int top_left_y = detection.box.y;
            int center_x = top_left_x+static_cast<int>(width/2);
            int center_y = top_left_y+static_cast<int>(height/2);
            std::pair<int, int> centroid = {center_x, center_y};
            centroids.push_back(centroid);
            RCLCPP_INFO_STREAM(get_logger(), "CENTROID X: " << center_x << " CENTROID Y: " << center_y);
        }
        return centroids;
    }

    auto LightDetector::caching(const std::vector<std::pair<int, int>> &centroids) -> void{
        double shortest_distance = std::numeric_limits<double>::infinity();
        std::pair<double, double> closest;
        //RCLCPP_INFO_STREAM(get_logger(),"num points: " << mHitCounts.size());
        for(auto const& [point, hc] : mHitCounts){
            //RCLCPP_INFO_STREAM(get_logger(),"current point: " << point.first << ", " << point.second << ": " << hc);
            double distance;
            if(hc >= mPublishThreshold){
                RCLCPP_INFO_STREAM(get_logger(),"Found a new point");
                RCLCPP_INFO_STREAM(get_logger(),"This new point was at " << point.first << ", " << point.second);
                distance = calculateDistance(point);
                if(distance <= shortest_distance){
                    shortest_distance = distance;
                    closest = point;
                }
            }
        }
    }

    auto LightDetector::calculateDistance(const std::pair<double, double> &p) -> double{
        double distance = sqrt(pow(p.first, 2) + pow(p.second, 2));
        //RCLCPP_INFO_STREAM(get_logger(),"Calculating Distance..." << p.first << ", " << p.second << " DISTANCE: " << distance);
        return distance;
    }

    auto LightDetector::getHitCount(std::optional<SE3d> const& light) -> int {
        if(light.has_value()){
            SE3d cameraToMap = SE3Conversions::fromTfTree(mTfBuffer, mCameraFrame, mWorldFrame);

            SE3d lightInMap = cameraToMap * light.value();

            auto location = lightInMap.translation();
            auto x = round_to(static_cast<double>(location.x()), 0.1);
            auto y = round_to(static_cast<double>(location.y()), 0.1);

            std::pair<double, double> key{x, y};

            return mHitCounts[key];
        }
        return -1;
    }

    void LightDetector::increaseHitCount(std::optional<SE3d> const& light){
        if(light.has_value()){
            SE3d cameraToMap = SE3Conversions::fromTfTree(mTfBuffer, mCameraFrame, mWorldFrame);
            SE3d lightInMap = cameraToMap * light.value();

            auto location = lightInMap.translation();
            auto x = round_to(static_cast<double>(location.x()), 0.1);
            auto y = round_to(static_cast<double>(location.y()), 0.1);

            std::pair<double, double> key{x, y};
            mHitCounts[key] = std::min(mHitCounts[key] + mHitIncrease, mHitMax);
            //mHitCounts[key] += 2;
        }
    }

    void LightDetector::decreaseHitCounts(){
        for(auto& [_, hitCount] : mHitCounts){
            hitCount = std::max(hitCount - mHitDecrease, 0);
            //hitCount--; 
        }
    }

    void LightDetector::printHitCounts(){
        // for(auto const& [key, val] : mHitCounts){
        //     //RCLCPP_INFO_STREAM(get_logger(),"Key: ( " << key.first << ", " << key.second << ") Val: " << val);
        // }
    }

    auto LightDetector::publishDebugObjects(cv::InputArray const& image) -> void {
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


    auto LightDetector::publishDetectedObjects(cv::InputArray image, std::vector<std::pair<int, int>> const& centroids) -> void {
        //if (!imgPub.getNumSubscribers()) return;

		sensor_msgs::msg::Image imgMsg;

        imgMsg.header.stamp = this->get_clock()->now(); //ros origionally, but changed to this, not sure if it works.
        imgMsg.height = image.rows();
        imgMsg.width = image.cols();
        imgMsg.encoding = sensor_msgs::image_encodings::BGRA8;
        imgMsg.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        imgMsg.step = 4 * imgMsg.width;
        imgMsg.data.resize(imgMsg.step * imgMsg.height);

		// Load the data into the message in the correct format
        cv::Mat debugImageWrapper{image.size(), CV_8UC4, imgMsg.data.data()};
        cv::cvtColor(image, debugImageWrapper, cv::COLOR_RGB2BGRA);

		// Draw a marker where each centroid is in the image
		constexpr int MARKER_RADIUS = 10;
		cv::Scalar const MARKER_COLOR = {255, 0, 0, 0};

		for(auto const& centroid : centroids){
			cv::circle(debugImageWrapper, {centroid.second, centroid.first}, MARKER_RADIUS, MARKER_COLOR);
		}

        imgPub->publish(imgMsg);
    }

    auto LightDetector::publishClosestLight(std::pair<double, double> &point) -> void {
        geometry_msgs::msg::Vector3 pointMsg;
        pointMsg.x = point.first;
        pointMsg.y = point.second;
        pointMsg.z = 0;
        RCLCPP_INFO_STREAM(get_logger(),"Publishing " << point.first << " , " << point.second);
        pointPub->publish(pointMsg);
    }


    auto LightDetector::spiralSearchForValidPoint(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& cloudPtr, std::size_t u, std::size_t v, std::size_t width, std::size_t height)const -> std::optional<SE3d> {
        // See: https://stackoverflow.com/a/398302
        int xc = static_cast<int>(u), yc = static_cast<int>(v);
        int sw = static_cast<int>(width), sh = static_cast<int>(height);
        int ih = static_cast<int>(cloudPtr->height), iw = static_cast<int>(cloudPtr->width);
        int sx = 0, sy = 0; // Spiral coordinates starting at (0, 0)
        int dx = 0, dy = -1;
        std::size_t bigger = std::max(width, height);
        std::size_t maxIterations = bigger * bigger;
        for (std::size_t i = 0; i < maxIterations; i++) {
            if (-sw / 2 < sx && sx <= sw / 2 && -sh / 2 < sy && sy <= sh / 2) {
                int ix = xc + sx, iy = yc + sy; // Image coordinates
                
                // Outside Image
                if (ix < 0 || ix >= iw || iy < 0 || iy >= ih) {
                    //rclcpp::RCLCPP_(std::format("Spiral query is outside the image: [{}, {}]", ix, iy));
                    continue;
                }

                // Could be error cause of double instead of int
                cv::Point3d const& point = reinterpret_cast<cv::Point3d const*>(cloudPtr->data.data())[ix + iy * cloudPtr->width];
                if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
                RCLCPP_INFO_STREAM(get_logger(),"X: " << point.x << " Y: " << point.y << " Z: " << point.z);
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
} //mrover
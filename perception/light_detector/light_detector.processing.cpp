#include "light_detector.hpp"
#include <cfloat>
#include <cmath>
#include <execution>
#include <manif/impl/se3/SE3.h>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>

namespace mrover {
    auto LightDetector::round_to(double value, double precision) -> double{
        double val = std::round(value / precision) * precision;
        return (floor((val*2)+0.5)/2);
    }

    auto LightDetector::caching() -> std::pair<std::pair<double, double>, bool>{
        double shortest_distance = std::numeric_limits<double>::infinity();
        bool found = false;
        std::pair<double, double> closest;
        for(auto const& [point, hc] : mHitCounts){
            double distance;
            if(hc >= mPublishThreshold){ 
                found = true;
                RCLCPP_INFO_STREAM(get_logger(),"Found a new point");
                RCLCPP_INFO_STREAM(get_logger(),"This new point was at " << point.first << ", " << point.second);
                distance = calculateDistance(point);
                if(distance <= shortest_distance){
                    shortest_distance = distance;
                    closest = point;
                }
            }
        }
        return std::pair<std::pair<double, double>, bool>(closest, found);
    }
    
    auto ColoredDetector::pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) -> void {
        // Adjust the picture size to be in line with the expected img size from the Point Cloud
        if (static_cast<int>(msg->height) != mImgRGB.rows || static_cast<int>(msg->width) != mImgRGB.cols) {
            RCLCPP_INFO_STREAM(get_logger(), std::format("Image size changed from [{}, {}] to [{}, {}]", mImgRGB.cols, mImgRGB.rows, msg->width, msg->height));
            mImgRGB = cv::Mat{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3, cv::Scalar{0, 0, 0, 0}};
        }

        // if 0x0 image
        if (!mImgRGB.total()) {
            return;
        }
        
		convertPointCloudToRGB(msg, mImgRGB);

        // Convert the RGB Image into the blob Image format
        cv::Mat blobSizedImage;
        mModel.rgbImageToBlob(mModel, mImgRGB, blobSizedImage, mImageBlob);

        // Run the blob through the model
        std::vector<Detection> detections{};
        cv::Mat outputTensor;
        mTensorRT.modelForwardPass(mImageBlob, outputTensor);

        mModel.outputTensorToDetections(mModel, outputTensor, detections);

        // Increment Object hit counts if theyre seen
        // Decrement Object hit counts if they're not seen
        updateHitsObject(msg, detections);

        // Draw the bounding boxes on the image
        drawDetectionBoxes(blobSizedImage, detections);
        if (mDebug) {
            publishDetectedObjects(blobSizedImage);
        }
        // NEED TO IMPLEMENT SOME SORT OF POINT PUB
	}

    auto ColoredDetector::convertPointCloudToRGB(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg, cv::Mat const& image) -> void {
        auto* pixelPtr = reinterpret_cast<cv::Vec3b*>(image.data);
        auto* pointPtr = reinterpret_cast<Point const*>(msg->data.data());
        std::for_each(std::execution::par_unseq, pixelPtr, pixelPtr + image.total(), [&](cv::Vec3b& pixel) {
            std::size_t const i = &pixel - pixelPtr;
            pixel[0] = pointPtr[i].r;
            pixel[1] = pointPtr[i].g;
            pixel[2] = pointPtr[i].b;
        });
    }

    auto ColoredDetector::drawDetectionBoxes(cv::InputOutputArray image, std::span<Detection const> detections) -> void {
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

    // Might Work Already, objectHitCounts stored in mModel instead of Hit Count Map
    auto ColoredDetector::updateHitsObject(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg, std::span<Detection const> detections, cv::Size const& imageSize) -> void {
        // Set of flags indicating if the given object has been seen
        std::vector<bool> seenObjects(mModel.classes.size(), false);
        for (auto const& [classId, className, confidence, box]: detections) {
            // Resize from blob space to image space
            cv::Point2f centerInBlob = cv::Point2f{box.tl()} + cv::Point2f{box.size()} / 2;
            float xRatio = static_cast<float>(msg->width) / static_cast<float>(imageSize.width);
            float yRatio = static_cast<float>(msg->height) / static_cast<float>(imageSize.height);
            std::size_t centerXInImage = std::lround(centerInBlob.x * xRatio);
            std::size_t centerYInImage = std::lround(centerInBlob.y * yRatio);

            // DO NOT KNOW IF THIS IS NEEDED
            // Get the object's position in 3D from the point cloud and run this statement if the optional has a value
            if (std::optional<SE3d> objectInCamera = spiralSearchForValidPoint(msg, centerXInImage, centerYInImage, box.width, box.height)) {
                try {
                    std::string objectImmediateFrame = std::format("immediate{}", className);
                    // Push the immediate detections to the camera frame
                    SE3Conversions::pushToTfTree(mTfBroadcaster, objectImmediateFrame, mCameraFrame, objectInCamera.value(), get_clock()->now());
                    // Since the object is seen we need to increment the hit counter
                    mModel.objectHitCounts[classId] = std::min(mHitMax, mModel.objectHitCounts[classId] + mHitIncrease);

                    // Only publish to permament if we are confident in the object
                    if (mModel.objectHitCounts[classId] > mPublishThreshold) {
                        std::string objectPermanentFrame = className;
                        // Grab the object inside of the camera frame and push it into the map frame
                        SE3d objectInMap = SE3Conversions::fromTfTree(mTfBuffer, objectImmediateFrame, mWorldFrame);
                        SE3Conversions::pushToTfTree(mTfBroadcaster, objectPermanentFrame, mWorldFrame, objectInMap, get_clock()->now());
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
            mModel.objectHitCounts[i] = std::max(0, mModel.objectHitCounts[i] - mHitDecrease);
        }
    }

    auto ColoredDetector::publishDetectedObjects(cv::InputArray image) -> std::optional<std::pair<double,double>> {
        mDetectionsImageMessage.header.stamp = get_clock()->now();
        mDetectionsImageMessage.height = image.rows();
        mDetectionsImageMessage.width = image.cols();
        mDetectionsImageMessage.encoding = sensor_msgs::image_encodings::BGRA8;
        mDetectionsImageMessage.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        mDetectionsImageMessage.step = 4 * mDetectionsImageMessage.width;
        mDetectionsImageMessage.data.resize(mDetectionsImageMessage.step * mDetectionsImageMessage.height);
        cv::Mat debugImageWrapper{image.size(), CV_8UC4, mDetectionsImageMessage.data.data()};
        cv::cvtColor(image, debugImageWrapper, cv::COLOR_RGB2BGRA);

        imgPub->publish(mDetectionsImageMessage);

        // Point Pub

        std::vector<bool> seenObjects(mModel.classes.size(), false);
        double shortest_distance = std::numeric_limits<double>::infinity();
        bool found = false;
        std::pair<double, double> closest;

        for (int i = 0; i < seenObjects.size(); i++) {
            if (seenObjects[i]) {
                double distance;
                if( >= mPublishThreshold){
                    found = true;
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

        geometry_msgs::msg::Vector3 pointMsg;
        pointMsg.x = point.first;
        pointMsg.y = point.second;
        pointMsg.z = 0;
        RCLCPP_INFO_STREAM(get_logger(),"Publishing " << point.first << " , " << point.second);
        pointPub->publish(pointMsg);

        double shortest_distance = std::numeric_limits<double>::infinity();
        bool found = false;
        std::pair<double, double> closest;
        //RCLCPP_INFO_STREAM(get_logger(),"num points: " << mHitCounts.size());
        for(auto const& [point, hc] : mHitCounts){
            //RCLCPP_INFO_STREAM(get_logger(),"current point: " << point.first << ", " << point.second << ": " << hc);
            
        }
        return std::pair<std::pair<double, double>, bool>(closest, found);


    }

    auto InfraredDetector::getPointFromPointCloud(sensor_msgs::msg::Image::ConstSharedPtr const& cloudPtr, std::pair<int, int> coordinates) -> std::optional<SE3d>{
        Point const& p = reinterpret_cast<Point const*>(cloudPtr->data.data())[coordinates.second + coordinates.first * cloudPtr->width];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)){
            //RCLCPP_INFO_STREAM(get_logger(),"failed");
            return std::nullopt;
        }
        RCLCPP_INFO_STREAM(get_logger(),"x: " << p.x << " y: " << p.y << " z: " << p.z);
        return std::make_optional<SE3d>(R3d{p.x, p.y, p.z}, SO3d::Identity());
    }

    auto InfraredDetector::convertImageToMat(sensor_msgs::msg::Image::ConstSharedPtr const& msg, cv::Mat const& image) -> void {
        auto* pixelPtr = reinterpret_cast<cv::Vec3b*>(image.data);
        auto* pointPtr = reinterpret_cast<Point const*>(msg->data.data());
        std::for_each(std::execution::par_unseq, pixelPtr, pixelPtr + image.total(), [&](cv::Vec3b& pixel) {
            std::size_t const i = &pixel - pixelPtr;
            pixel[0] = pointPtr[i].r;
            pixel[1] = pointPtr[i].g;
            pixel[2] = pointPtr[i].b;
        });
    }

    // Ignore these hit count stuff for now, gonna change them completely
    auto InfraredDetector::getHitCount(std::optional<SE3d> const& light) -> int {
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

    void InfraredDetector::increaseHitCount(std::optional<SE3d> const& light){
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

    void InfraredDetector::decreaseHitCounts(){
        for(auto& [_, hitCount] : mHitCounts){
            hitCount = std::max(hitCount - mHitDecrease, 0);
        }
    }

    auto InfraredDetector::imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void {
        if(mImgRGB.rows != static_cast<int>(msg->height) || mImgRGB.cols != static_cast<int>(msg->width)) {
			RCLCPP_INFO_STREAM(get_logger(),"Adjusting Image Size... " << msg->width << ", " << msg->height);
		    mImgRGB = cv::Mat{cv::Size{static_cast<int>(msg->width), static_cast<int>(msg->height)}, CV_8UC3, {0, 0, 0}};
			mThresholdedImg = cv::Mat{cv::Size{static_cast<int>(msg->width), static_cast<int>(msg->height)}, CV_8UC1, cv::Scalar{0}};
        }
        
		convertImageToMat(msg, mImgRGB);

        //applies a gaussian blur and thresholding to mGreyScale
        cv::Mat mGBlur;
        cv::Mat mThresholdedImg;
        cv::GaussianBlur(mImgRGB, mGBlur, cv::Size(5,5), 0);
        cv::threshold(mGBlur,mThresholdedImg,100,255,cv::THRESH_BINARY);
        // cv::threshold(mGBlur,mThresholdedImg,100,255,cv::THRESH_BINARY+cv::THRESH_OTSU);

        // applies dilation, uses structuring element to expand highlighted regions
        // erodes, to refine shape of highlighted regions
        // dilates again to accentuate features
        cv::Mat erode;

        cv::dilate(mThresholdedImg, erode, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2,2), cv::Point(-1,-1)), cv::Point(-1,-1), cv::BORDER_REFLECT_101, 0);

        cv::erode(erode, mThresholdedImg, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2,2), cv::Point(-1,-1)), cv::Point(-1,-1), cv::BORDER_REFLECT_101, 0);

        cv::dilate(mThresholdedImg, erode, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3), cv::Point(-1,-1)), cv::Point(-1,-1), cv::BORDER_REFLECT_101, 0);
		
        // // converts erode into BGRA format and stores in mOutputImage
        cv::cvtColor(erode, mOutputImage, cv::COLOR_GRAY2BGRA);
		
        // finds contours in eroded image and stores in "contours"
        // contour hierarchy information stored in hierarchy
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(erode, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

		// Find the centroids for all of the different contours
        // quick question: why not store as cv::point? 
		std::vector<std::pair<int, int>> centroids; // These are in image space
		centroids.resize(contours.size());

        //RCLCPP_INFO_STREAM(get_logger(),"Number of contours " << contours.size());
    

		for(std::size_t i = 0; i < contours.size(); ++i){
			auto const& vec = contours[i];
			auto& centroid = centroids[i]; // first = row, second = col

            // find avg of the contour for each contour for midpoint
			for(auto const& point : vec){
				centroid.first += point.y;
				centroid.second += point.x;
            }

			// This is protected division since vec will only exist if it contains points
			centroid.first /= static_cast<int>(vec.size());
			centroid.second /= static_cast<int>(vec.size());

            // If the position of the light is defined, then push it into the TF tree
            //std::optional<SE3d> lightInCamera = spiralSearchForValidPoint(msg, centroid.second, centroid.first, SPIRAL_SEARCH_DIM, SPIRAL_SEARCH_DIM);
            
            std::optional<SE3d> lightInCamera = getPointFromPointCloud(msg, centroid);
            if(lightInCamera.has_value()){
                auto lightTranslation = lightInCamera.value().translation();
                int x = static_cast<int>(lightTranslation.x());
                int y = static_cast<int>(lightTranslation.y()); 
                int z = static_cast<int>(lightTranslation.z()); 
                //RCLCPP_INFO_STREAM(get_logger(),"X: " << x << " Y: " << y << " Z: " << z);  
            }
            
            if(lightInCamera){ 
                ++numLightsSeen;
                std::string immediateLightFrame = std::format("immediateLight{}", numLightsSeen);
                if(lightInCamera.value().translation().norm() < mImmediateLightRange && getHitCount(lightInCamera) > mPublishThreshold) {
                    std::string lightFrame = std::format("light{}", numLightsSeen);
                    SE3Conversions::pushToTfTree(mTfBroadcaster, lightFrame, mCameraFrame, lightInCamera.value(), this->get_clock()->now());
                }
                increaseHitCount(lightInCamera);
                //RCLCPP_INFO_STREAM(get_logger(),getHitCount(lightInCamera));
                SE3Conversions::pushToTfTree(mTfBroadcaster, immediateLightFrame, mCameraFrame, lightInCamera.value(), this->get_clock()->now());
            }
		}

        decreaseHitCounts();

        std::pair<std::pair<double, double>, bool> foundPoint = caching();
        if(foundPoint.second){
            publishClosestLight(foundPoint.first);
        }

		publishDetectedObjects(mOutputImage, centroids);
	}

    auto InfraredDetector::publishDetectedObjects(cv::InputArray image, std::vector<std::pair<int, int>> const& centroids) -> void {
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

    // Might use for colored detector,
    auto LightDetector::publishClosestLight(std::pair<double, double> &point) -> void {
        geometry_msgs::msg::Vector3 pointMsg;
        pointMsg.x = point.first;
        pointMsg.y = point.second;
        pointMsg.z = 0;
        RCLCPP_INFO_STREAM(get_logger(),"Publishing " << point.first << " , " << point.second);
        pointPub->publish(pointMsg);
    }

    auto LightDetector::calculateDistance(const std::pair<double, double> &p) -> double{
        double distance = sqrt(pow(p.first, 2) + pow(p.second, 2));
        //RCLCPP_INFO_STREAM(get_logger(),"Calculating Distance..." << p.first << ", " << p.second << " DISTANCE: " << distance);
        return distance;
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
#include "light_detector.hpp"
#include <cfloat>
#include <cmath>
#include <execution>
#include <manif/impl/se3/SE3.h>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>

namespace mrover {
    // TODO: (john) break this out into a utility so we dont have to copy all of this code
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

    auto LightDetector::getPointFromPointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& cloudPtr, std::pair<int, int> coordinates) -> std::optional<SE3d>{
        /* auto* pointPtr = reinterpret_cast<Point const*>(msg->data.data());
        RCLCPP_INFO_STREAM(get_logger(),"SIZE: " << msg->data.size());
        auto msgWidth = msg->width;
        RCLCPP_INFO_STREAM(get_logger(),"WIDTH: " << msg->width);
        RCLCPP_INFO_STREAM(get_logger(),"HEIGHT: " << msg->height);
        RCLCPP_INFO_STREAM(get_logger(),"X: " << coordinates.second << " Y: " << coordinates.first);
         */
        Point const& p = reinterpret_cast<Point const*>(cloudPtr->data.data())[coordinates.second + coordinates.first * cloudPtr->width];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)){
            //RCLCPP_INFO_STREAM(get_logger(),"failed");
            return std::nullopt;
        }
        RCLCPP_INFO_STREAM(get_logger(),"x: " << p.x << " y: " << p.y << " z: " << p.z);
        return std::make_optional<SE3d>(R3d{p.x, p.y, p.z}, SO3d::Identity());
    }
    
    auto LightDetector::imageCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) -> void {
        // resizing image is mImgRGB dimensions doesn't match msg dimensions
        // initialize mThresholdedImg to grayscale image 
        if(mImgRGB.rows != static_cast<int>(msg->height) || mImgRGB.cols != static_cast<int>(msg->width)) {
			RCLCPP_INFO_STREAM(get_logger(),"Adjusting Image Size... " << msg->width << ", " << msg->height);
		    mImgRGB = cv::Mat{cv::Size{static_cast<int>(msg->width), static_cast<int>(msg->height)}, CV_8UC3, {0, 0, 0}};
			mThresholdedImg = cv::Mat{cv::Size{static_cast<int>(msg->width), static_cast<int>(msg->height)}, CV_8UC1, cv::Scalar{0}};
        }
        
		convertPointCloudToRGB(msg, mImgRGB);

        // conversion to grayscale and storing in mGreyScale - (added)
        cv::Mat mGreyScale;
        cv::cvtColor(mImgRGB, mGreyScale, cv::COLOR_RGB2GRAY);

        //applies a gaussian blur and thresholding to mGreyScale
        cv::Mat mGBlur;
        cv::Mat mThresholdedImg;
        cv::GaussianBlur(mGreyScale, mGBlur, cv::Size(5,5), 0);
        cv::threshold(mGBlur,mThresholdedImg,50,255,cv::THRESH_BINARY);
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
                RCLCPP_INFO_STREAM(get_logger(),getHitCount(lightInCamera));
                SE3Conversions::pushToTfTree(mTfBroadcaster, immediateLightFrame, mCameraFrame, lightInCamera.value(), this->get_clock()->now());
            }
		}

        //printHitCounts();

        decreaseHitCounts();

        std::pair<std::pair<int, int>, bool> foundPoint = caching();
        if(foundPoint.second){
            publishClosestLight(foundPoint.first);
        }

		publishDetectedObjects(mOutputImage, centroids);
	}
    
    auto LightDetector::caching() -> std::pair<std::pair<int, int>, bool>{
        double shortest_distance = std::numeric_limits<double>::infinity();
        bool found = false;
        std::pair<int, int> closest;
        //RCLCPP_INFO_STREAM(get_logger(),"num points: " << mHitCounts.size());
        for(auto const& [point, hc] : mHitCounts){
            //RCLCPP_INFO_STREAM(get_logger(),"current point: " << point.first << ", " << point.second << ": " << hc);
            double distance;
            if(hc >= 10){ //DANTODO: ASK appropriate hitcount requirement to be considered a light
                found = true;
                distance = calculateDistance(point);
                if(distance <= shortest_distance){
                    shortest_distance = distance;
                    closest = point;
                    //RCLCPP_INFO_STREAM(get_logger(),"Found a new point!, distance was " << distance);
                    //RCLCPP_INFO_STREAM(get_logger(),"This new point was at " << point.first << ", " << point.second);
                }
            }
            // if(found){
            //     //RCLCPP_INFO_STREAM(get_logger(),"Finished with a new point!");
            // } else{
            //     //RCLCPP_INFO_STREAM(get_logger(),"Didn't find a new point :(");
            // }
        }
        return std::pair<std::pair<int, int>, bool>(closest, found);
    }

    auto LightDetector::calculateDistance(const std::pair<int, int> &p) -> double{
        double distance = sqrt(pow(p.first, 2) + pow(p.second, 2));
        //RCLCPP_INFO_STREAM(get_logger(),"Calculating Distance..." << p.first << ", " << p.second);
        return distance;
    }

    auto LightDetector::getHitCount(std::optional<SE3d> const& light) -> int {
        if(light.has_value()){
            SE3d cameraToMap = SE3Conversions::fromTfTree(mTfBuffer, mCameraFrame, mWorldFrame);

            SE3d lightInMap = cameraToMap * light.value();

            auto location = lightInMap.translation();
            int x = static_cast<int>(location.x());
            int y = static_cast<int>(location.y());

            std::pair<int, int> key{x, y};

            return mHitCounts[key];
        }
        return -1;
    }

    void LightDetector::increaseHitCount(std::optional<SE3d> const& light){
        if(light.has_value()){
            SE3d cameraToMap = SE3Conversions::fromTfTree(mTfBuffer, mCameraFrame, mWorldFrame);
            
            /* auto lightLocation = light.value().translation();
            int locx = static_cast<int>(lightLocation.x());
            int locy = static_cast<int>(lightLocation.y());
            RCLCPP_INFO_STREAM(get_logger(),"current point: " << locx << ", " << locy);
            */
            
            SE3d lightInMap = cameraToMap * light.value();

            auto location = lightInMap.translation();
            int x = static_cast<int>(location.x());
            int y = static_cast<int>(location.y());

            std::pair<int, int> key{x, y};
            //RCLCPP_INFO_STREAM(get_logger(),"current point: " << x << ", " << y);
            mHitCounts[key] = std::min(mHitCounts[key] + mHitIncrease, mHitMax);  //DANTODO why are values being stored as int pairs instead of more precise measures?
        }
    }

    //DANTODO: ASK
    void LightDetector::decreaseHitCounts(){
        for(auto& [_, hitCount] : mHitCounts){
            hitCount = std::max(hitCount - mHitDecrease, 0);
        }
    }

    void LightDetector::printHitCounts(){
        // for(auto const& [key, val] : mHitCounts){
        //     //RCLCPP_INFO_STREAM(get_logger(),"Key: ( " << key.first << ", " << key.second << ") Val: " << val);
        // }
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

    auto LightDetector::publishClosestLight(std::pair<int, int> &point) -> void {
        geometry_msgs::msg::Vector3 pointMsg;
        pointMsg.x = point.first;
        pointMsg.y = point.second;
        pointMsg.z = 0;
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
#include "light_detector.hpp"
#include <cfloat>
#include <cmath>
#include <execution>
#include <manif/impl/se3/SE3.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>

namespace mrover {
    // TODO: (john) break this out into a utility so we dont have to copy all of this code
	auto LightDetector::convertPointCloudToRGB(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg, cv::Mat const& image) -> void {
        //Outputs BGR image
        auto* pixelPtr = reinterpret_cast<cv::Vec3b*>(image.data);
        auto* pointPtr = reinterpret_cast<Point const*>(msg->data.data());
        std::for_each(std::execution::par_unseq, pixelPtr, pixelPtr + image.total(), [&](cv::Vec3b& pixel) {
            std::size_t const i = &pixel - pixelPtr;
            //cv::Mat default is bgr
            pixel[0] = pointPtr[i].b;
            pixel[1] = pointPtr[i].g;
            pixel[2] = pointPtr[i].r;
        });
    }

    /* auto LightDetector::round_to(double value, double precision) -> double{
        double val = std::round(value / precision) * precision;
        return (floor((val*2)+0.5)/2);
    } */

    //could use a short here maybe
    auto withinRange(uint32_t value, uint32_t color, uint32_t tolerance) -> bool{
        if((value <= color + tolerance) && (value >= color-tolerance)){
            return true;
        } else{
            return false;
        }
    }

    auto LightDetector::setLightModeCallback(mrover::srv::SetLightMode::Request::ConstSharedPtr& request,
         mrover::srv::SetLightMode::Response::SharedPtr& response) -> void{
            RCLCPP_INFO_STREAM(get_logger(),"service callback");
            if(request->color == "red"){
                mColorBound[0] = 205; //r
                mColorBound[1] = 50;  //g
                mColorBound[2] = 50;  //b
                response->color = "red";
            } else if(request->color == "blue"){
                mColorBound[0] = 50; //r
                mColorBound[1] = 50;  //g
                mColorBound[2] = 205;  //b
                response->color = "blue";
            } else{
                response->color = "undef";
            }
         }

    auto LightDetector::getPointFromPointCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& cloudPtr, std::pair<int, int> coordinates) -> std::optional<SE3d>{
        Point const& p = reinterpret_cast<Point const*>(cloudPtr->data.data())[coordinates.second + coordinates.first * cloudPtr->width];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)){
            //pointcloud value at this location did not exist
            return std::nullopt;
        }
        return std::make_optional<SE3d>(R3d{p.x, p.y, p.z}, SO3d::Identity());
    }
    
    auto LightDetector::imageCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) -> void {
        
        //Resize RGB image to msg if sizes aren't equal
        if(mImgRGB.rows != static_cast<int>(msg->height) || mImgRGB.cols != static_cast<int>(msg->width)) {
		    mImgRGB = cv::Mat{cv::Size{static_cast<int>(msg->width), static_cast<int>(msg->height)}, CV_8UC3, {0, 0, 0}};
			mThresholdedImg = cv::Mat{cv::Size{static_cast<int>(msg->width), static_cast<int>(msg->height)}, CV_8UC1, cv::Scalar{0}};
        }
             
		convertPointCloudToRGB(msg, mImgRGB);

        if(!mImgRGB.empty()){
            cv::imwrite("output.png", mImgRGB);
        }

        // conversion to grayscale and storing in mGreyScale
        cv::Mat mGreyScale;
        cv::cvtColor(mImgRGB, mGreyScale, cv::COLOR_BGR2GRAY);
        cv::imwrite("grey_output.png", mGreyScale);

        // Applies gaussian blur and thresholding to mGreyScale
        cv::Mat mGBlur;
        cv::Mat mThresholdedImg;
        cv::GaussianBlur(mGreyScale, mGBlur, cv::Size(15,15), 0);
        cv::imwrite("gaussian_output.png", mGBlur);

        cv::threshold(mGBlur,mThresholdedImg,100,255,cv::THRESH_BINARY);
        cv::imwrite("thresh_output.png", mThresholdedImg);

        // applies dilation, uses structuring element to expand highlighted regions
        // erodes, to refine shape of highlighted regions
        // dilates again to accentuate features
        cv::Mat dilate;
        cv::dilate(mThresholdedImg, dilate, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2,2), cv::Point(-1,-1)), cv::Point(-1,-1), cv::BORDER_REFLECT_101, 0);
        cv::imwrite("dilated_first_output.png", dilate);

        cv::Mat erode;
        cv::erode(dilate, erode, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2,2), cv::Point(-1,-1)), cv::Point(-1,-1), cv::BORDER_REFLECT_101, 0);
        cv::imwrite("eroded_first_output.png", erode);

        cv::dilate(erode, dilate, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3), cv::Point(-1,-1)), cv::Point(-1,-1), cv::BORDER_REFLECT_101, 0);
        cv::imwrite("dilated_second_output.png", dilate);

        // // converts erode into BGRA format and stores in mOutputImage
        cv::cvtColor(erode, mOutputImage, cv::COLOR_GRAY2BGRA);
        cv::imwrite("eroded_stored.png", erode);        
		
        // finds contours in eroded grayscale image and stores in "contours"
        std::vector<std::vector<cv::Point>> contours;

        // Finds contours from eroded image. RETR_LIST does not store any hierarchy information. 
        // cv::CHAIN_APPROX_SIMPLE stores only useful points on a contour. For example, a rectangle would only have four points.  
        // daniel TODO: potentially flawed teardrop shape
		cv::findContours(erode, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

		// Find the centroids for all of the different contours 
		std::vector<std::pair<int, int>> centroids;
		centroids.resize(contours.size());

        RCLCPP_INFO_STREAM(get_logger(),"Number of contours " << contours.size());
    

		for(std::size_t i = 0; i < contours.size(); ++i){
			auto const& vec = contours[i];
            // first = row, second = col
			auto& centroid = centroids[i]; 

            // find avg of the contour for each contour for midpoint, not necessarily exactly what we want
			for(auto const& point : vec){
				centroid.first += point.y;
				centroid.second += point.x;
            }

			// This is protected division since vec will only exist if it contains points
			centroid.first /= static_cast<int>(vec.size());
			centroid.second /= static_cast<int>(vec.size());

            // traverse ZED message in a spiral starting from the centroid in a 50x50 dimensional radius and returning the first point that actually has a ZED point cloud value 
            std::optional<SE3d> lightInCamera = spiralSearchForValidPoint(msg, centroid.second, centroid.first, SPIRAL_SEARCH_DIM, SPIRAL_SEARCH_DIM);
            
            RCLCPP_INFO_STREAM(get_logger(), "centroid x: " << centroid.first << " centroid y: " << centroid.second);
            RCLCPP_INFO_STREAM(get_logger(), "RGB dims x: " << mImgRGB.rows << " RGB dims y: " << mImgRGB.cols);
            RCLCPP_INFO_STREAM(get_logger(), "tolerance: " << mColorBound.size());

            if(lightInCamera.has_value() && withinRange(mImgRGB.at<cv::Vec3b>(centroid.first, centroid.second)[0], mColorBound[2], mColorBound[3])
            && withinRange(mImgRGB.at<cv::Vec3b>(centroid.first, centroid.second)[1], mColorBound[1], mColorBound[3]) 
            && withinRange(mImgRGB.at<cv::Vec3b>(centroid.first, centroid.second)[2], mColorBound[0], mColorBound[3])){ 
                ++numLightsSeen;
                if(lightInCamera.value().translation().norm() < mImmediateLightRange) {
                    increaseHitCount(lightInCamera.value());
                }
            }
		}

        std::optional<std::pair<double, double>> foundPoint = getClosestPoint();

        decreaseHitCounts();

        if(foundPoint.has_value()){
            publishLight(foundPoint.value());
        }
		//publishDetectedObjects(mOutputImage, centroids);
	}

    auto LightDetector::getClosestPoint() -> std::optional<std::pair<double, double>>{
        double shortest_distance = std::numeric_limits<double>::infinity();
        bool foundPoint = false;
        std::pair<double, double> closest;
        for(auto const& [point, hc] : mHitCounts){
            int distance;
            if(hc >= mPublishThreshold){
                distance = calculateDistance(point);
                if(distance <= shortest_distance){
                    shortest_distance = distance;
                    closest = point;
                    foundPoint = true;
                }
            }
        }
        if (foundPoint){
            return closest;
        } else{
            return std::nullopt;
        }
    }

    auto LightDetector::calculateDistance(const std::pair<double, double> &p) -> double{
        double distance = sqrt(pow(p.first, 2) + pow(p.second, 2));
        return distance;
    }

/*     auto LightDetector::getHitCount(std::optional<SE3d> const& light) -> int {
        if(light.has_value()){
            SE3d cameraToMap = SE3Conversions::fromTfTree(mTfBuffer, mCameraFrame, mMapFrame);

            SE3d lightInMap = cameraToMap * light.value();

            auto location = lightInMap.translation();
            auto x = round_to(static_cast<double>(location.x()), 0.1);
            auto y = round_to(static_cast<double>(location.y()), 0.1);

            std::pair<double, double> key{x, y};

            return mHitCounts[key];
        }
        return -1;
    } */

    void LightDetector::increaseHitCount(SE3d const& light){
            SE3d cameraToMap = SE3Conversions::fromTfTree(mTfBuffer, mCameraFrame, mMapFrame);
            SE3d lightInMap = cameraToMap * light;
            //RCLCPP_INFO_STREAM(get_logger(),"ORIGINAL SE3D " << light.translation().x() << " , " << light.translation().y());
            RCLCPP_INFO_STREAM(get_logger(),"TRANSLATED SE3D " << lightInMap.translation().x() << " , " << lightInMap.translation().y());
            //RCLCPP_INFO_STREAM(get_logger(),"CAMERATOMAP SE3D " << cameraToMap.translation().x() << " , " << cameraToMap.translation().y());

            auto t = lightInMap.translation();
            auto x = std::round(t.x());
            auto y = std::round(t.y());
            std::pair<double, double> key{x, y};
            mHitCounts[key] = std::min(mHitCounts[key] + mHitIncrease, mHitMax);
    }

    void LightDetector::decreaseHitCounts(){
        for(auto& [_, hitCount] : mHitCounts){
            //daniel TODO: Does this have potential to fix see-saw problem?
            hitCount = std::max(hitCount - mHitDecrease, 0); 
        }
    }

    auto LightDetector::publishDetectedObjects(cv::InputArray image, std::vector<std::pair<int, int>> const& centroids) -> void {

		sensor_msgs::msg::Image imgMsg;

        imgMsg.header.stamp = this->get_clock()->now(); //ros originally, but changed to this, not sure if it works.
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

    auto LightDetector::publishLight(std::pair<double, double> &point) -> void {
        geometry_msgs::msg::Vector3 pointMsg;
        pointMsg.x = point.first;
        pointMsg.y = point.second;
        pointMsg.z = 0;
        RCLCPP_INFO_STREAM(get_logger(),"Publishing " << point.first << " , " << point.second);
        pointPub->publish(pointMsg);
    }


    auto LightDetector::spiralSearchForValidPoint(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& cloudPtr, std::size_t u, std::size_t v, std::size_t width, std::size_t height)const -> std::optional<SE3d> {
        // daniel TODO: figure out why this actually works
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
#include "perception.hpp"
#include "mrover/msg/detail/starter_project_tag__struct.hpp"


// ROS Headers, ros namespace
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);


    // "spin" blocks until our node dies
    rclcpp::spin(std::make_shared<mrover::Perception>());
    rclcpp::shutdown();


    return EXIT_SUCCESS;
}


namespace mrover {


    Perception::Perception() : Node("perception") {
        // Subscribe to camera image messages
        // Every time another node publishes to this topic we will be notified
        // Specifically the callback we passed will be invoked
        mImageSubscriber = create_subscription<sensor_msgs::msg::Image>("zed/left/image", 1, [this](sensor_msgs::msg::Image::ConstSharedPtr const& msg) {
            imageCallback(msg);
        });


        // Create a publisher for our tag topic
        // See: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
        // TODO: uncomment me!
        mTagPublisher = create_publisher<msg::StarterProjectTag>("tag", 1);


        mTagDictionary = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));
    }


    auto Perception::imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& imageMessage) -> void {
        // Create a cv::Mat from the ROS image message
        // Note this does not copy the image data, it is basically a pointer
        // Be careful if you extend its lifetime beyond this function
        cv::Mat imageBGRA{static_cast<int>(imageMessage->height), static_cast<int>(imageMessage->width),
                          CV_8UC4, const_cast<uint8_t*>(imageMessage->data.data())};
        cv::Mat image;


        cv::cvtColor(imageBGRA, image, cv::COLOR_BGRA2BGR);


        // TODO: implement me!
        // hint: think about the order in which these functions were implemented ;)
        std::vector<msg::StarterProjectTag> tags;
        findTagsInImage(image, tags);

        imageBGRA.deallocate();

        if (tags.empty()) {
            image.deallocate();
            return;
        }

        image.deallocate();
        publishTag(selectTag(image, tags));
    }


    auto Perception::findTagsInImage(cv::Mat const& image, std::vector<msg::StarterProjectTag>& tags) -> void { // NOLINT(*-convert-member-functions-to-static)
        // hint: take a look at OpenCV's documentation for the detectMarkers function
        // hint: you have mTagDictionary, mTagCorners, and mTagIds member variables already defined! (look in perception.hpp)
        // hint: write and use the "getCenterFromTagCorners" and "getClosenessMetricFromTagCorners" functions
        tags.clear(); // Clear old tags in output vector
        cv::aruco::detectMarkers(image, mTagDictionary, mTagCorners, mTagIds);
        tags.resize(mTagCorners.size());
        for (uint32_t i = 0; i < tags.size(); i++) {
            // set tag ids
            tags[i].set__tag_id(mTagIds[i]);


            // set center points
            std::pair<float, float> center;
            center = getCenterFromTagCorners(mTagCorners[i]);
            tags[i].set__x_tag_center_pixel(center.first);
            tags[i].set__x_tag_center_pixel(center.second);


            // set closeness
            tags[i].set__closeness_metric(getClosenessMetricFromTagCorners(image, mTagCorners[i]));
        }
    }


    auto Perception::selectTag(cv::Mat const& image, std::vector<msg::StarterProjectTag> const& tags) -> msg::StarterProjectTag { // NOLINT(*-convert-member-functions-to-static)
        double minDist = std::numeric_limits<double>::max();
        msg::StarterProjectTag minDistTag;
        std::pair<float, float> img_center = {image.rows / 2, image.cols / 2};


        for (auto tag: tags) {
            double tagDist = sqrt(pow((tag.x_tag_center_pixel - img_center.first), 2) + pow((tag.y_tag_center_pixel - img_center.second), 2));
            if (tagDist < minDist) {
                minDist = tagDist;
                minDistTag = tag;
            }
        }
        return minDistTag;
    }


    auto Perception::publishTag(msg::StarterProjectTag const& tag) -> void {
        mTagPublisher->publish(tag);
    }


    auto Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) -> float { // NOLINT(*-convert-member-functions-to-static)
        // hint: think about how you can use the "image" parameter
        // hint: this is an approximation that will be used later by navigation to stop "close enough" to a tag.
        // hint: try not overthink, this metric does not have to be perfectly accurate, just correlated to distance away from a tag
        std::pair<float, float> center = getCenterFromTagCorners(tagCorners);
        float tagArea = 4 * (tagCorners[0].x - center.first) * (tagCorners[0].y - center.second);
        uint32_t imageArea = image.cols * image.rows;
        return tagArea / (float) imageArea;
    }


    auto Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) -> std::pair<float, float> { // NOLINT(*-convert-member-functions-to-static)
        std::pair<float, float> center;
        for (auto tagCorner: tagCorners) {
            center = {(center.first + tagCorner.x) / 4, (center.second + tagCorner.y) / 4};
        }
        return center;
    }


} // namespace mrover
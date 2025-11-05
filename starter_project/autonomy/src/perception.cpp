#include "perception.hpp"
#include "mrover/msg/detail/starter_project_tag__struct.hpp"

// ROS Headers, ros namespace
#include <cmath>
#include <functional>
#include <iostream>
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
#include <tuple>
#include <utility>

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

        findTagsInImage(image,mTags);
        msg::StarterProjectTag tag;
        tag = selectTag(image,mTags);
        publishTag(tag);
    }

    auto Perception::findTagsInImage(cv::Mat const& image, std::vector<msg::StarterProjectTag>& tags) -> void { // NOLINT(*-convert-member-functions-to-static)
        // hint: take a look at OpenCV's documentation for the detectMarkers function
        // hint: you have mTagDictionary, mTagCorners, and mTagIds member variables already defined! (look in perception.hpp)
        // hint: write and use the "getCenterFromTagCorners" and "getClosenessMetricFromTagCorners" functions

        tags.clear(); // Clear old tags in output vector

        cv::aruco::detectMarkers(image, mTagDictionary, mTagCorners, mTagIds);

        std::pair<float, float> centers;
        float closeness = 0;

        for(unsigned long i=0; i<mTagIds.size(); ++i) {
            centers = getCenterFromTagCorners(mTagCorners[i]);
            closeness = getClosenessMetricFromTagCorners(image, mTagCorners[i]);

            msg::StarterProjectTag detected_tag;
            detected_tag.set__tag_id(mTagIds[i]);
            detected_tag.set__x_tag_center_pixel(centers.first);
            detected_tag.set__y_tag_center_pixel(centers.second);
            detected_tag.set__closeness_metric(closeness);
            tags.push_back(detected_tag);
        }

    }

    auto Perception::selectTag(cv::Mat const& image, std::vector<msg::StarterProjectTag> const& tags) -> msg::StarterProjectTag { // NOLINT(*-convert-member-functions-to-static)
        float max = 0;
        msg::StarterProjectTag default_tag;
        default_tag.set__tag_id(0l);
        default_tag.set__x_tag_center_pixel(0.0f);
        default_tag.set__y_tag_center_pixel(0.0f);
        default_tag.set__closeness_metric(0.0f);
        if(tags.size() != 0) {
            max = tags[0].closeness_metric;
            int max_index = 0;
            for(unsigned long i=0; i<tags.size(); ++i) {
                if(tags[i].closeness_metric > max) {
                    max_index = i;
                }
            }

            return msg::StarterProjectTag{tags[max_index]};
        }
        else {
            return default_tag;
        }
    }

    auto Perception::publishTag(msg::StarterProjectTag const& tag) -> void {
        mTagPublisher->publish(tag);
    }

    auto Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) -> float { // NOLINT(*-convert-member-functions-to-static)
        // hint: think about how you can use the "image" parameter
        // hint: this is an approximation that will be used later by navigation to stop "close enough" to a tag.
        // hint: try not overthink, this metric does not have to be perfectly accurate, just correlated to distance away from a tag

        float x_tag_range = tagCorners[1].x - tagCorners[0].x;

        float image_width = image.cols;

        return x_tag_range / image_width;
    }

    auto Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) -> std::pair<float, float> { // NOLINT(*-convert-member-functions-to-static)

        float x_tag_center_pixel = ((tagCorners[1].x - tagCorners[0].x) / 2.0) + tagCorners[0].x;
        float y_tag_center_pixel = ((tagCorners[2].y - tagCorners[1].y) / 2.0) + tagCorners[1].y;

        std::pair<float, float> centers = std::make_pair(x_tag_center_pixel,y_tag_center_pixel);
        return {centers};
    }

} // namespace mrover
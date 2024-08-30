#include "perception.hpp"

// ROS Headers, ros namespace
#include <cmath>
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

        // Detect tags in the image pixels
        findTagsInImage(image, mTags);
        // Select the tag that is closest to the middle of the screen
        msg::StarterProjectTag tag = selectTag(image, mTags);
        // Publish the message to our topic so navigation or others can receive it
        publishTag(tag);
    }

    auto Perception::findTagsInImage(cv::Mat const& image, std::vector<msg::StarterProjectTag>& tags) -> void { // NOLINT(*-convert-member-functions-to-static)
        // hint: take a look at OpenCV's documentation for the detectMarkers function
        // hint: you have mTagDictionary, mTagCorners, and mTagIds member variables already defined! (look in perception.hpp)
        // hint: write and use the "getCenterFromTagCorners" and "getClosenessMetricFromTagCorners" functions

        tags.clear(); // Clear old tags in output vector

        // TODO: implement me!

        cv::aruco::detectMarkers(image, mTagDictionary, mTagCorners, mTagIds);

        unsigned int numTagsDetected = mTagCorners.size();

        for(unsigned int i  = 0; i < numTagsDetected; ++i){
            auto center = getCenterFromTagCorners(mTagCorners[i]);
            auto closeness = getClosenessMetricFromTagCorners(image, mTagCorners[i]);
            
            msg::StarterProjectTag newTag;
            newTag.tag_id = mTagIds[i];
            newTag.x_tag_center_pixel = center.first;
            newTag.y_tag_center_pixel = center.second;
            newTag.closeness_metric = closeness;

            tags.push_back(newTag);
        }
    }

    auto Perception::selectTag(cv::Mat const& image, std::vector<msg::StarterProjectTag> const& tags) -> msg::StarterProjectTag { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!

        std::pair<float, float> center{image.cols / 2.0, image.rows / 2.0};

        float closestDistance = std::numeric_limits<float>::infinity();
        msg::StarterProjectTag closestTag{};

        for(auto const& tag : tags){
            std::pair<float, float> tagCenter{tag.x_tag_center_pixel, tag.y_tag_center_pixel};

            auto distFromImageCenter = static_cast<float>(std::sqrt(std::pow((tagCenter.first - center.first), 2) + std::pow((tagCenter.second - center.second), 2)));

            closestTag = (distFromImageCenter < closestDistance) ? tag : closestTag;
            closestDistance = (distFromImageCenter < closestDistance) ? distFromImageCenter : closestDistance;
        }

        return closestTag;
    }

    auto Perception::publishTag(msg::StarterProjectTag const& tag) -> void {
        // TODO: implement me!

        mTagPublisher->publish(tag);
    }

    auto Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) -> float { // NOLINT(*-convert-member-functions-to-static)
        // hint: think about how you can use the "image" parameter
        // hint: this is an approximation that will be used later by navigation to stop "close enough" to a tag.
        // hint: try not overthink, this metric does not have to be perfectly accurate, just correlated to distance away from a tag

        // TODO: implement me!

        // Side length (Not perfect because it may not be parrallel with the image plane)

        // This works because the ordering os clockwise
        auto length = static_cast<float>(std::sqrt(std::pow(tagCorners[0].x - tagCorners[3].x, 2) + std::pow(tagCorners[0].y - tagCorners[3].y, 2)));

        return length / static_cast<float>(image.rows);
    }

    auto Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) -> std::pair<float, float> { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!

        cv::Point2f sum = std::accumulate(std::begin(tagCorners), std::end(tagCorners), cv::Point2f{0, 0});
        sum /= 4;
        
        return {sum.x, sum.y};
    }

} // namespace mrover
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
        for (size_t i = 0; i < mTagIds.size(); ++i) {
            // For each tag detected, in the image, create a new StarterProjectTag and add it to the mTags vector
            msg::StarterProjectTag tag;
            tag.tag_id = mTagIds[i];
            std::pair<float, float> center = getCenterFromTagCorners(mTagCorners[i]);
            // Express each tag center to be relative to the center of the image
            // This is to make calculating the center tag in selectTag possible and it makes navigation easier down the line
            // Ask an auton lead if you have questions about this!
            // std::cout << center.first << " " << center.second << std::endl;
            tag.x_tag_center_pixel = (center.first - float(image.cols) / 2) / float(image.cols);
            tag.y_tag_center_pixel = (center.second - float(image.rows) / 2) / float(image.rows);
            tag.closeness_metric = getClosenessMetricFromTagCorners(image, mTagCorners[i]);
            mTags.push_back(tag);
        }
    }

    auto Perception::selectTag(cv::Mat const& image, std::vector<msg::StarterProjectTag> const& tags) -> msg::StarterProjectTag { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!
        if (!tags.empty()) {
            if (tags.size() == 1) {
                // If only one tag is seen, return it
                return tags[0];
            }
            int minTagIndex = 0;
            double minTagDist = sqrt(pow(tags[0].x_tag_center_pixel, 2) + pow(tags[0].y_tag_center_pixel, 2));
            // For each tag seen, if its distance to the origin is less than that of the previous tags, it is now the selected tag
            for (int i = 1; i < tags.size(); ++i) {
                double currentTagDist = sqrt(pow(tags[i].x_tag_center_pixel, 2) + pow(tags[i].y_tag_center_pixel, 2));
                if (currentTagDist < minTagDist) {
                    minTagDist = currentTagDist;
                    minTagIndex = i;
                }
            }
            return tags[minTagIndex];

        } else {
            // If no tag is seen, return a tag with a false (-1) indicator
            msg::StarterProjectTag tag;
            tag.tag_id = -1;
            return tag;
        }
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
        // Find three of the corners to calculate the area of the tag from
        // We are assuming the tag is a square
        float imageSize = float(image.cols) * float(image.rows);
        cv::Point2f topLeft = tagCorners[0];
        cv::Point2f topRight = tagCorners[1];
        cv::Point2f bottomLeft = tagCorners[2];

        // Calculate the area of the tag
        float tagWidth = topRight.x - topLeft.x;
        float tagHeight = bottomLeft.y - topLeft.y;

        // Metric is the ratio between the tag area and total image area
        float metric = abs(tagWidth * tagHeight) / imageSize;
        // Metric goes from 0 to 1 where 0 means really close and 1 means really far
        // When the tag is really close, it takes up a lot of area on the screen and the ratio is closer to 1, the inverse of what we want
        // Thus we must return 1 - metric
        return 1 - metric;
    }

    auto Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) -> std::pair<float, float> { // NOLINT(*-convert-member-functions-to-static)
        // TODO: implement me!
        float xSum = 0;
        float ySum = 0;
        // The center is the sum of the x and y coordinates of the corners
        for (auto& corner: tagCorners) {
            xSum += corner.x;
            ySum += corner.y;
        }
        std::pair<float, float> center(xSum / 4.0, ySum / 4.0);
        return center;
    }

} // namespace mrover
#include "perception.hpp"
#include "mrover/msg/detail/starter_project_tag__struct.hpp"

// ROS Headers, ros namespace
#include <cmath>
#include <cstddef>
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

        // Simply call findTagsInImage to fill in the tag vector and then publish the closest one
        findTagsInImage(image, mTags);
        publishTag(selectTag(image, mTags));
    }

    auto Perception::findTagsInImage(cv::Mat const& image, std::vector<msg::StarterProjectTag>& tags) -> void { // NOLINT(*-convert-member-functions-to-static)
        // hint: take a look at OpenCV's documentation for the detectMarkers function
        // hint: you have mTagDictionary, mTagCorners, and mTagIds member variables already defined! (look in perception.hpp)
        // hint: write and use the "getCenterFromTagCorners" and "getClosenessMetricFromTagCorners" functions

        tags.clear(); // Clear old tags in output vector

        // Outputs tagIDs and tagCorners in respective vectors, they will be the same size
        cv::aruco::detectMarkers(image, mTagDictionary, mTagCorners, mTagIds);

        // Given tag corners, we need to calculate tag centers and place into tags vector
        for(size_t i = 0; i < mTagCorners.size(); i++){
            // Get tag center, tag id, closeness metric from our functions and the tag id vector
            int tagID = mTagIds[i];
            std::pair<float, float> tag_center = getCenterFromTagCorners(mTagCorners[i]);
            float closenessMetric = getClosenessMetricFromTagCorners(image, mTagCorners[i]);

            // Declare a new tag and fill in the relevant information. Add it to the tag vector
            msg::StarterProjectTag tag;
            tag.tag_id = tagID;
            tag.x_tag_center_pixel = tag_center.first;
            tag.y_tag_center_pixel = tag_center.second;
            tag.closeness_metric = closenessMetric;
            tags.push_back(tag);
        }
    }

    auto Perception::selectTag(cv::Mat const& image, std::vector<msg::StarterProjectTag> const& tags) -> msg::StarterProjectTag { // NOLINT(*-convert-member-functions-to-static)
        // Find largest closeness metric
        msg::StarterProjectTag closest;
        closest.closeness_metric = 0;
        for(auto tag : tags){
            if(tag.closeness_metric > closest.closeness_metric){
                closest = tag;
            }
        }
        
        return closest;
    }

    auto Perception::publishTag(msg::StarterProjectTag const& tag) -> void {
        // Simply publish the passed tag
        mTagPublisher->publish(tag);
    }

    auto Perception::getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) -> float { // NOLINT(*-convert-member-functions-to-static)
        // hint: think about how you can use the "image" parameter
        // hint: this is an approximation that will be used later by navigation to stop "close enough" to a tag.
        // hint: try not overthink, this metric does not have to be perfectly accurate, just correlated to distance away from a tag

        // The closer a tag is, the more of the camera it takes up, so the larger the tag area is, the larger
        //     its closeness metric should be
        // If the tag takes up the entire screen, it should return 1 as the closeness metric
        cv::Point2f topLeft = tagCorners[0];
        cv::Point2f bottomRight = tagCorners[2];

        float area = abs(topLeft.x - bottomRight.x) * abs(topLeft.y - bottomRight.y);
        area /= (float)(image.cols * image.rows);
        return area;
    }

    auto Perception::getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) -> std::pair<float, float> { // NOLINT(*-convert-member-functions-to-static)
        // To calculate the center, we only need two corners, the top left and bottom right
        // The corners are listed clockwise starting from the top left
        cv::Point2f topLeft = tagCorners[0];
        cv::Point2f bottomRight = tagCorners[2];

        float x_center = (topLeft.x + bottomRight.x) / 2.0f;
        float y_center = (topLeft.y + bottomRight.y) / 2.0f;
        return {x_center, y_center};
    }

} // namespace mrover
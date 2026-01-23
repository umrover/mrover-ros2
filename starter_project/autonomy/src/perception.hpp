#pragma once

// C++ Standard Library Headers, std namespace
#include <memory>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

// OpenCV Headers, cv namespace
#include <opencv2/aruco.hpp>
#include <opencv2/core/mat.hpp>

// ROS Headers, ros namespace
# include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <mrover/msg/starter_project_tag.hpp>

// #if __has_include(<mrover/msg/starter_project_tag.hpp>)
// #include <mrover/StarterProjectTag.h>
// #else
// struct StarterProjectTag {};
// #endif

namespace mrover {

    /**
     *  Starter project perception node
     *
     *  Input:  Image data, just RGB pixels.
     *  Output: ArUco tag pixel coordinates that is closest to the center of the camera.
     *          Also an approximation for how far away the tag is.
     */
    class Perception : public rclcpp::Node {
    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr mImageSubscriber;
        cv::Ptr<cv::aruco::Dictionary> mTagDictionary;
        std::vector<std::vector<cv::Point2f>> mTagCorners;
        std::vector<int> mTagIds;
        std::vector<msg::StarterProjectTag> mTags;
        rclcpp::Publisher<msg::StarterProjectTag>::SharedPtr mTagPublisher;

    public:
        Perception();

        /**
         * Called when we receive a new image message from the camera.
         * Specifically this is one frame.
         *
         * @param imageMessage
         */
        void imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& imageMessage);

        /**
         *  Given an image, detect ArUco tags, and fill a vector full of output messages.
         *
         * @param image Image
         * @param tags  Output vector of tags
         */
        void findTagsInImage(cv::Mat const& image, std::vector<msg::StarterProjectTag>& tags);

        /**
         * Publish our processed tag
         *
         * @param tag Selected tag message
         */
        void publishTag(msg::StarterProjectTag const& tag);

        /**
         *  Given an ArUco tag in pixel space, find a metric for how close we are.
         *
         * @param image         Access to the raw OpenCV image as a matrix
         * @param tagCorners    4-tuple of the tag pixel coordinates representing the corners
         * @return              Closeness metric from rover to the tag
         */
        [[nodiscard]] auto getClosenessMetricFromTagCorners(cv::Mat const& image, std::vector<cv::Point2f> const& tagCorners) -> float;

        /**
         *  Given an ArUco tag in pixel space, find the approximate center in pixel space
         *
         * @param tagCorners    4-tuple of tag pixel coordinates representing the corners
         * @return              2-tuple (x,y) approximate center in pixel space
         */
        [[nodiscard]] auto getCenterFromTagCorners(std::vector<cv::Point2f> const& tagCorners) -> std::pair<float, float>;

        /**
         *  Select the tag closest to the center of the camera
         * 
         * @param tags          Vector of tags
         * @return              Center tag
         */
        [[nodiscard]] auto selectTag(cv::Mat const& image, std::vector<msg::StarterProjectTag> const& tags) -> msg::StarterProjectTag;
    };

} // namespace mrover
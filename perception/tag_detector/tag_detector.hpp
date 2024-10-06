#pragma once

#include "pch.hpp"

namespace mrover {

    struct Tag {
        int id = -1;
        int hitCount = 0;
        cv::Point2f imageCenter{};
        std::optional<SE3d> tagInCam;
    };

    class TagDetectorNodeletBase : public rclcpp::Node {

    protected:
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mDetectedImagePub;
        std::unordered_map<int, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> mThreshImagePubs; // Map from threshold scale to publisher

        std::string mMapFrameId;
        std::string mCameraFrameId;
        int mMinTagHitCountBeforePublish;
        int mMaxTagHitCount;
        int mTagIncrementWeight;
        int mTagDecrementWeight;

        tf2_ros::Buffer mTfBuffer{get_clock()};
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster{this};

        cv::Ptr<cv::aruco::DetectorParameters> mDetectorParams;
        cv::Ptr<cv::aruco::Dictionary> mDictionary;

        cv::Mat mBgrImage, mGrayImage;
        sensor_msgs::msg::Image mDetectionsImageMessage, mThresholdImageMessage;
        std::optional<std::size_t> mPrevDetectedCount; // Log spam prevention
        std::vector<std::vector<cv::Point2f>> mImmediateCorners;
        std::vector<int> mImmediateIds;
        std::unordered_map<int, Tag> mTags;

        LoopProfiler mProfiler{get_logger()};

        auto publishThresholdedImage() -> void;

        auto publishDetectedTags() -> void;

    public:
        explicit TagDetectorNodeletBase(std::string const& name);

        ~TagDetectorNodeletBase() override = default;
    };

    class StereoTagDetectorNodelet final : public TagDetectorNodeletBase {

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mPointCloudSub;

        auto pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) -> void;

        auto spiralSearchForValidPoint(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& cloudPtr,
                                       std::size_t u, std::size_t v,
                                       std::size_t width, std::size_t height) const -> std::optional<SE3d>;

    public:
        StereoTagDetectorNodelet();
    };

    class ImageTagDetectorNodelet final : public TagDetectorNodeletBase {

        float mCameraHorizontalFOV;

        rclcpp::Publisher<msg::ImageTargets>::SharedPtr mTargetsPub;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSub;

        auto getTagBearing(cv::InputArray image, std::span<cv::Point2f const> tagCorners) const -> float;

        auto imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void;

    public:
        ImageTagDetectorNodelet();
    };

} // namespace mrover

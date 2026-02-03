#include "tag_detector.hpp"

namespace mrover {

    Combiner::Combiner(std::string const& name, rclcpp::NodeOptions const& options) : Node{name, options} {
        std::string cameraName{};
        std::vector<ParameterWrapper> params{
                {"camera_name", cameraName, "camera"}};

        std::string imageTopicName = std::format("/{}/image", cameraName);
        std::string tagTopicName = std::format("/{}/tag_boxes", cameraName);
        std::string malletTopicName = std::format("/{}/mallet_boxes", cameraName);
        std::string bottleTopicName = std::format("/{}/bottle_boxes", cameraName);

        ParameterWrapper::declareParameters(this, params);
    
        mImageSub = create_subscription<sensor_msgs::msg::Image>(imageTopicName, 1, [this](sensor_msgs::msg::Image::ConstSharedPtr const& msg){
                    imageCallback(msg);
                });

        mTagBoxesSub = create_subscription<mrover::msg::ObjectBoundingBoxes>(tagTopicName, 1, [this](mrover::msg::ObjectBoundingBoxes const& msg){
                    mTagBoxes = msg;
                });

        mMalletBoxesSub = create_subscription<mrover::msg::ObjectBoundingBoxes>(malletTopicName, 1, [this](mrover::msg::ObjectBoundingBoxes const& msg){
                    mMalletBoxes = msg;
                });

        mBottleBoxesSub = create_subscription<mrover::msg::ObjectBoundingBoxes>(bottleTopicName, 1, [this](mrover::msg::ObjectBoundingBoxes const& msg){
                    mBottleBoxes = msg;
                });
    }

    auto Combiner::imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void{
        cv::Mat bgraImage{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, const_cast<std::uint8_t*>(msg->data.data())}; // No copy is made, it simply wraps the pointer
        cv::Mat bgrImage;
        cv::cvtColor(bgraImage, bgrImage, cv::COLOR_BGRA2BGR);

        for(auto const& tag : mTagBoxes.targets){
            cv::line(bgrImage, cv::Point(static_cast<int>(tag.x), static_cast<int>(tag.y)), cv::Point(static_cast<int>(tag.x), static_cast<int>(tag.y + tag.h)), cv::Scalar(0, 255, 0), 2, cv::LINE_4);
            cv::line(bgrImage, cv::Point(static_cast<int>(tag.x), static_cast<int>(tag.y + tag.h)), cv::Point(static_cast<int>(tag.x + tag.w), static_cast<int>(tag.y + tag.h)), cv::Scalar(0, 255, 0), 2, cv::LINE_4);
            cv::line(bgrImage, cv::Point(static_cast<int>(tag.x + tag.w), static_cast<int>(tag.y + tag.h)), cv::Point(static_cast<int>(tag.x + tag.w), static_cast<int>(tag.y)), cv::Scalar(0, 255, 0), 2, cv::LINE_4);
            cv::line(bgrImage, cv::Point(static_cast<int>(tag.x + tag.w), static_cast<int>(tag.y)), cv::Point(static_cast<int>(tag.x), static_cast<int>(tag.y)), cv::Scalar(0, 255, 0), 2, cv::LINE_4);
        }

        mImage.header.stamp = get_clock()->now();
        mImage.header.frame_id = "zed_left_camera_frame";
        mImage.height = msg->height;
        mImage.width = msg->width;
        mThresholdImageMessage.encoding = sensor_msgs::image_encodings::MONO8;
        mThresholdImageMessage.step = mGrayImage.step;
        mThresholdImageMessage.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        size_t size = mThresholdImageMessage.step * mThresholdImageMessage.height;
        mThresholdImageMessage.data.resize(size);
        std::memcpy(mThresholdImageMessage.data.data(), mGrayImage.data, size);

        publisher->publish(mThresholdImageMessage);
    }
} // namespace mrover

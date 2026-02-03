#include "tag_detector.hpp"

namespace mrover {

    Combiner::Combiner(rclcpp::NodeOptions const& options) : Node{"combiner", options} {
        std::string cameraName{};
        std::vector<ParameterWrapper> params{
                {"camera_name", cameraName, "camera"}};

        ParameterWrapper::declareParameters(this, params);

        std::string imageTopicName = std::format("/{}_cam/image", cameraName);
        std::string tagTopicName = std::format("/{}_cam/tag_boxes", cameraName);
        std::string malletTopicName = std::format("/{}_cam/mallet_boxes", cameraName);
        std::string bottleTopicName = std::format("/{}_cam/bottle_boxes", cameraName);
        std::string imagePubTopicName = std::format("/{}_cam/detections", cameraName);
    
        mImageSub = create_subscription<sensor_msgs::msg::Image>(imageTopicName, 1, [this](sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void{
                    imageCallback(msg);
                });

        mTagBoxesSub = create_subscription<mrover::msg::ObjectBoundingBoxes>(tagTopicName, 1, [this](mrover::msg::ObjectBoundingBoxes const& msg) -> void{
                    mTagBoxes = msg;
                });

        mMalletBoxesSub = create_subscription<mrover::msg::ObjectBoundingBoxes>(malletTopicName, 1, [this](mrover::msg::ObjectBoundingBoxes const& msg) -> void{
                    mMalletBoxes = msg;
                });

        mBottleBoxesSub = create_subscription<mrover::msg::ObjectBoundingBoxes>(bottleTopicName, 1, [this](mrover::msg::ObjectBoundingBoxes const& msg) -> void{
                    mBottleBoxes = msg;
                });

        mImagePub = create_publisher<sensor_msgs::msg::Image>(imagePubTopicName, 1);
    }

    auto Combiner::imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void{
        cv::Mat bgraImage{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, const_cast<std::uint8_t*>(msg->data.data())}; // No copy is made, it simply wraps the pointer

        for(auto const& tag : mTagBoxes.targets){
            cv::line(bgraImage, cv::Point(static_cast<int>(tag.x), static_cast<int>(tag.y)), cv::Point(static_cast<int>(tag.x), static_cast<int>(tag.y + tag.h)), cv::Scalar(0, 255, 0), 2, cv::LINE_4);
            cv::line(bgraImage, cv::Point(static_cast<int>(tag.x), static_cast<int>(tag.y + tag.h)), cv::Point(static_cast<int>(tag.x + tag.w), static_cast<int>(tag.y + tag.h)), cv::Scalar(0, 255, 0), 2, cv::LINE_4);
            cv::line(bgraImage, cv::Point(static_cast<int>(tag.x + tag.w), static_cast<int>(tag.y + tag.h)), cv::Point(static_cast<int>(tag.x + tag.w), static_cast<int>(tag.y)), cv::Scalar(0, 255, 0), 2, cv::LINE_4);
            cv::line(bgraImage, cv::Point(static_cast<int>(tag.x + tag.w), static_cast<int>(tag.y)), cv::Point(static_cast<int>(tag.x), static_cast<int>(tag.y)), cv::Scalar(0, 255, 0), 2, cv::LINE_4);
        }

        mImage.header.stamp = get_clock()->now();
        mImage.header.frame_id = "zed_left_camera_frame";
        mImage.height = msg->height;
        mImage.width = msg->width;
        mImage.encoding = sensor_msgs::image_encodings::BGRA8;
        mImage.step = msg->step;
        mImage.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        size_t size = msg->step * msg->height;
        mImage.data.resize(size);
        std::memcpy(mImage.data.data(), bgraImage.data, size);

        mImagePub->publish(mImage);
    }
} // namespace mrover

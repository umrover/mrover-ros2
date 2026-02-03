#include "combiner.hpp"

namespace mrover {

    Combiner::Combiner(rclcpp::NodeOptions const& options) : Node{"combiner", options} {
        std::string cameraName{};
        std::vector<ParameterWrapper> params{
                {"camera_name", cameraName, "camera"}};

        ParameterWrapper::declareParameters(this, params);

        std::string imageTopicName = std::format("/{}_cam/image", cameraName);
        std::string tagTopicName = std::format("/{}_cam/tag_boxes", cameraName);
        std::string objectTopicName = std::format("/{}_cam/object_boxes", cameraName);
        std::string imagePubTopicName = std::format("/{}_cam/detections", cameraName);
    
        mImageSub = create_subscription<sensor_msgs::msg::Image>(imageTopicName, 1, [this](sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void{
                    imageCallback(msg);
                });

        mTagBoxesSub = create_subscription<mrover::msg::TagBoundingBoxes>(tagTopicName, 1, [this](mrover::msg::TagBoundingBoxes const& msg) -> void{
                    mTagCorners.clear();
                    mTagIds.clear();
                    for(auto const& tag : msg.targets){
                        assert(tag.corners.size() == 8); // there should be 4 pairs of points

                        std::vector<cv::Point2f> vec{};
                        for(std::size_t i = 0; i < 8; i += 2){
                            cv::Point2f p(tag.corners[i], tag.corners[i + 1]);
                            vec.push_back(p);
                        }
                        mTagCorners.push_back(vec);
                        mTagIds.push_back(tag.id);
                    }
                });

        mObjectBoxesSub = create_subscription<mrover::msg::ObjectBoundingBoxes>(objectTopicName, 1, [this](mrover::msg::ObjectBoundingBoxes const& msg) -> void{
                    mObjectBoxes = msg;
                });

        mImagePub = create_publisher<sensor_msgs::msg::Image>(imagePubTopicName, 1);
    }

    auto Combiner::imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void{
        cv::Mat bgraImage{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, const_cast<std::uint8_t*>(msg->data.data())}; // No copy is made, it simply wraps the pointer
        cv::Mat bgrImage{};
        cv::cvtColor(bgraImage, bgrImage, cv::COLOR_BGRA2BGR);
        cv::aruco::drawDetectedMarkers(bgrImage, mTagCorners, mTagIds);
        cv::cvtColor(bgrImage, bgraImage, cv::COLOR_BGR2BGRA);

        for(auto const& obj : mObjectBoxes.targets){
            cv::Vec3b color = getColor(obj.object);
            cv::line(bgraImage, cv::Point(static_cast<int>(obj.x), static_cast<int>(obj.y)), cv::Point(static_cast<int>(obj.x), static_cast<int>(obj.y + obj.h)), color, 2, cv::LINE_4);
            cv::line(bgraImage, cv::Point(static_cast<int>(obj.x), static_cast<int>(obj.y + obj.h)), cv::Point(static_cast<int>(obj.x + obj.w), static_cast<int>(obj.y + obj.h)), color, 2, cv::LINE_4);
            cv::line(bgraImage, cv::Point(static_cast<int>(obj.x + obj.w), static_cast<int>(obj.y + obj.h)), cv::Point(static_cast<int>(obj.x + obj.w), static_cast<int>(obj.y)), color, 2, cv::LINE_4);
            cv::line(bgraImage, cv::Point(static_cast<int>(obj.x + obj.w), static_cast<int>(obj.y)), cv::Point(static_cast<int>(obj.x), static_cast<int>(obj.y)), color, 2, cv::LINE_4);
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

    auto Combiner::getColor(std::string const& type) -> cv::Vec3b{
        static const cv::Vec3b orange(0, 132, 255);
        static const cv::Vec3b blue(255, 43, 0);
        if(type == "hammer"){
            return orange;
        }else if(type == "bottle"){
            return blue;
        }else{
            throw std::runtime_error(std::format("Unknown Class {}...", type));
        }
    }
} // namespace mrover

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::Combiner)

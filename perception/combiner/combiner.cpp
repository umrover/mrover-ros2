#include "combiner.hpp"

namespace mrover {
    std::array<std::pair<std::string, cv::Vec3b>, Combiner::NUM_CLASSES> const Combiner::CLASSES = {
        std::pair<std::string, cv::Vec3b>{"hammer", cv::Vec3b{0,132,255}},
        std::pair<std::string, cv::Vec3b>{"bottle", cv::Vec3b{255,43,0}}
    };

    Combiner::Combiner(rclcpp::NodeOptions const& options) : Node{"combiner", options} {
        std::string cameraName{};
        std::vector<ParameterWrapper> params{
                {"camera_name", cameraName, "camera"}};

        ParameterWrapper::declareParameters(this, params);

        std::string imageTopicName = std::format("/{}_cam/image", cameraName);
        std::string tagTopicName = std::format("/{}_cam/tag_boxes", cameraName);
        std::string objectTopicName = std::format("/{}_cam/object_boxes", cameraName);
        std::string imagePubTopicName = std::format("/{}_cam/detections", cameraName);

        for(auto const& [name, _] : CLASSES){
            mObjectBoxes[name] = std::tuple<std::deque<cv::Rect>, std::deque<float>, std::deque<std::size_t>>();
        }
    
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
                    for(auto& [name, qs] : mObjectBoxes){
                        auto& [bboxQ, confQ, numQ] = qs;
                        std::size_t numAdded = 0;
                        for(auto const& bbox : msg.targets){
                            if(bbox.object == name){
                                bboxQ.emplace_back(static_cast<int>(bbox.x), static_cast<int>(bbox.y), static_cast<int>(bbox.w), static_cast<int>(bbox.h));
                                confQ.push_back(0.5);
                                ++numAdded;
                            }
                        }

                        numQ.push_back(numAdded);

                        if(numQ.size() > WINDOW_SIZE){
                            for(std::size_t i = 0; i < numQ.front(); ++i){
                                bboxQ.pop_front();
                                confQ.pop_front();
                            }
                            numQ.pop_front();
                        }
                    }
                });

        mImagePub = create_publisher<sensor_msgs::msg::Image>(imagePubTopicName, 1);
    }

    auto Combiner::imageCallback(sensor_msgs::msg::Image::ConstSharedPtr const& msg) -> void{
        cv::Mat bgrImage{};
        {
            cv::Mat bgraImage{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, const_cast<std::uint8_t*>(msg->data.data())}; // No copy is made, it simply wraps the pointer
            cv::cvtColor(bgraImage, bgrImage, cv::COLOR_BGRA2BGR); // good from const cast
            cv::aruco::drawDetectedMarkers(bgrImage, mTagCorners, mTagIds);
        }

        for(auto const& [name, qs] : mObjectBoxes){
            cv::Vec3b const& color = getColor(name);
            auto const& [bboxQ, confQ, _] = qs;

            std::vector<int> indices{};
            std::vector<cv::Rect> const bboxVec(bboxQ.begin(), bboxQ.end());
            std::vector<float> const confVec(confQ.begin(), confQ.end());
            cv::dnn::NMSBoxes(bboxVec, confVec, 0.4, 0.2, indices);

            for(auto const& idx : indices){
                cv::rectangle(bgrImage, bboxVec[idx].tl(), bboxVec[idx].br(), color, LINE_THICKNESS);
            }
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
        // this allows for two copies instead of three
        cv::Mat msgImage{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, const_cast<std::uint8_t*>(mImage.data.data())}; // No copy is made, it simply wraps the pointer
        cv::cvtColor(bgrImage, msgImage, cv::COLOR_BGR2BGRA);

        mImagePub->publish(mImage);
    }

    auto Combiner::getColor(std::string const& type) -> cv::Vec3b{
        for(auto const& [name, color] : CLASSES){
            if(type == name){
                return color;
            }
        }

        throw std::runtime_error(std::format("Unknown Class {}...", type));
    }
} // namespace mrover

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::Combiner)

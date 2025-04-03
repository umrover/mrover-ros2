#pragma once

#include "pch.hpp"

namespace mrover {

    using PointCloudGpu = wrapped_thrust::thrust::device_vector<Point>;

    class ZedWrapper : public rclcpp::Node {
    private:
        static constexpr char const* NODE_NAME = "zed_wrapper";

        std::unique_ptr<tf2_ros::Buffer> mTfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        std::shared_ptr<tf2_ros::TransformListener> mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);

        struct Measures {
            rclcpp::Time time;
            sl::Mat leftImage;
            sl::Mat rightImage;
            sl::Mat leftPoints;
            sl::Mat leftNormals;

            Measures() = default;

            Measures(Measures&) = delete;
            auto operator=(Measures&) -> Measures& = delete;

            Measures(Measures&&) noexcept;
            auto operator=(Measures&&) noexcept -> Measures&;
        };

        LoopProfiler mLoopProfilerGrab;
        LoopProfiler mLoopProfilerUpdate;

        // Params
        int mSerialNumber{};
        int mGrabTargetFps{};
        int mDepthConfidence{};
        int mTextureConfidence{};

        bool mUseDepthStabilization{};
        bool mDepthEnabled{};
        bool mUseBuiltinPosTracking{};
        bool mUsePoseSmoothing{};
        bool mUseAreaMemory{};

        double mDepthMaximumDistance{};

        // CUDA
        PointCloudGpu mPointCloudGpu;

        // ZED
        sl::Camera mZed;
        sl::CameraInformation mZedInfo;

        Measures mGrabMeasures, mPcMeasures;

        sl::Resolution mImageResolution, mPointResolution, mNormalsResolution;

        sl::String mSvoPath;

        // Publishers
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mRightImgPub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mLeftImgPub;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr mImuPub;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mMagPub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPcPub;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr mLeftCamInfoPub;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr mRightCamInfoPub;
        rclcpp::Publisher<mrover::msg::Heading>::SharedPtr mMagHeadingPub;

        // Thread

        std::thread mPointCloudThread, mGrabThread;
        std::mutex mSwapMutex;
        std::condition_variable mSwapCv;
        bool mIsSwapReady = false;

        auto grabThread() -> void;

        auto pointCloudUpdateThread() -> void;

    public:
        explicit ZedWrapper(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

        ~ZedWrapper() override;
    };

    auto slTime2Ros(sl::Timestamp t) -> rclcpp::Time;

    auto fillImuMessage(rclcpp::Node* node, sl::SensorsData::IMUData& imuData, sensor_msgs::msg::Imu& msg) -> void;

    auto fillMagMessage(sl::SensorsData::MagnetometerData const& magData, sensor_msgs::msg::MagneticField& magMsg, mrover::msg::Heading &headingMsg) -> void;

    auto checkCudaError(cudaError_t error) -> void;

    void fillPointCloudMessageFromGpu(sl::Mat& xyzGpu, sl::Mat& bgraGpu, sl::Mat& normalsGpu, PointCloudGpu& pcGpu, sensor_msgs::msg::PointCloud2::UniquePtr const& msg);

    auto fillCameraInfoMessages(sl::CalibrationParameters& calibration, sl::Resolution const& resolution,
                                sensor_msgs::msg::CameraInfo& leftInfoMsg, sensor_msgs::msg::CameraInfo& rightInfoMsg) -> void;

    auto fillImageMessage(sl::Mat const& bgra, sensor_msgs::msg::Image::UniquePtr const& msg) -> void;
}; // namespace mrover

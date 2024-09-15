#pragma once

#include "pch.hpp"

namespace mrover {

    using Server = rclcpp_action::Server<action::LanderAlign>::SharedPtr;

    // {x, y, omega, dVelocity, dOmega}
    using Vector5d = Eigen::Matrix<double, 5, 1>;

    enum struct RTRSTATE {
        turn1 = 0,
        drive = 1,
        turn2 = 2,
        done = 3,
    };

    constexpr char RTRSTRINGS[4][6] = {
            "turn1",
            "drive",
            "turn2",
            "done ",
    };

    class LanderAlign final : public rclcpp::Node {
    private:
		static constexpr char const* NODE_NAME = "lander_align";

		//RANSAC VARS
        double mDistanceThreshold{};

        double mBestOffset{};

        double mPlaneOffsetScalar{};

        std::optional<Eigen::Vector3d> mPlaneLocationInZEDVector;
        std::optional<Eigen::Vector3d> mPlaneLocationInWorldVector;

        std::optional<Eigen::Vector3d> mOffsetLocationInZEDVector;
        std::optional<Eigen::Vector3d> mOffsetLocationInWorldVector;

        std::optional<Eigen::Vector3d> mNormalInZEDVector;
        std::optional<Eigen::Vector3d> mNormalInWorldVector;

        SE3d mOffsetLocationInWorldSE3d;
        SE3d mPlaneLocationInWorldSE3d;

        double mZThreshold{};
        double mXThreshold{};
        int mLeastSamplingDistribution = 10;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mDebugPCPub;

        std::vector<Point const*> mFilteredPoints;

        std::vector<Vector5d> mPathPoints;

		//RTR RTR VARS
        RTRSTATE mLoopState;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mTwistPub;

        //PID CONSTANTS
        double const mAngleP = 1;
        double const mAngleFloor = 0.05;
        double const mLinearP = 0.3;
        
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr mDebugVectorPub;

        //Action Server Variables
        sensor_msgs::msg::PointCloud2::ConstSharedPtr mCloud;

        std::optional<Server> mActionServer;

        //TF Member Variables
        std::unique_ptr<tf2_ros::Buffer> mTfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
        std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        std::shared_ptr<tf2_ros::TransformListener> mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);
        std::string mCameraFrameId;
        std::string mMapFrameId;

        //Ransac Params
        void filterNormals(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& cloud);

        void ransac(double distanceThreshold, int minInliers, int epochs);

        void sendTwist();

        void uploadPC(int numInliers, double distanceThreshold);

        void calcMotion(double desiredVelocity, double desiredOmega);

        void calcMotionToo();

        static auto calcAngleWithWorldX(Eigen::Vector3d xHeading) -> double;

        auto createSpline(int density, double offset) -> bool;

        void publishSpline();
    
    public:
        LanderAlign();

        ~LanderAlign() override;
	
        void ActionServerCallBack(std::shared_ptr<action::LanderAlign_Goal> goal);
    };

} // namespace mrover

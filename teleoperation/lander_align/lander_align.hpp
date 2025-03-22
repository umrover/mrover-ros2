#pragma once

#include "pch.hpp"
#include <rclcpp/subscription.hpp>
#include <rclcpp/wait_set.hpp>

namespace mrover {
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

        rclcpp::Time fuck = rclcpp::Time(0);
        
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
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mSubscriber;

        bool mReadPointCloud = true;

        std::vector<Point const*> mFilteredPoints;

        std::vector<Vector5d> mPathPoints;

		//RTR RTR VARS
        RTRSTATE mLoopState;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mTwistPub;
        geometry_msgs::msg::Twist driveTwist;
        geometry_msgs::msg::Twist turnTwist;

        //PID CONSTANTS
        double const mAngleP = 1;
        double const mAngleFloor = 0.05;
        double const mLinearP = 0.3;

        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr mDebugVectorPub;

        //Action Server Variables
        std::optional<sensor_msgs::msg::PointCloud2::ConstSharedPtr> mCloud;

        rclcpp::Service<mrover::srv::AlignLander>::SharedPtr mAction;

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


        using GoalUUID = rclcpp_action::GoalUUID;
        using GoalHandleLanderAlign = rclcpp_action::ServerGoalHandle<action::LanderAlign>;


        auto handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const action::LanderAlign::Goal> goal) -> rclcpp_action::GoalResponse;
        auto handle_cancel(std::shared_ptr<GoalHandleLanderAlign> goal_handle) -> rclcpp_action::CancelResponse;
        auto handle_accepted(std::shared_ptr<GoalHandleLanderAlign> goal_handle) -> void;
        auto execute(const std::shared_ptr<GoalHandleLanderAlign> goal_handle) -> void;
    
    public:
        LanderAlign();

        ~LanderAlign() override;
	
        void ActionServerCallBack(std::shared_ptr<GoalHandleLanderAlign> goal);

        void subscriberCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg);
    };

} // namespace mrover

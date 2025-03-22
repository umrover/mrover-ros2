#include "pch.hpp"
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/detail/float32__struct.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/logging.hpp>
#include <mrover/action/lander_align.hpp>
namespace mrover {
    class LanderAlignNode final : public rclcpp::Node{
        public:
        using LanderAlign = action::LanderAlign;
        using GoalHandleLanderAlign = rclcpp_action::ServerGoalHandle<LanderAlign>;

        explicit LanderAlignNode(rclcpp::NodeOptions const& options = rclcpp::NodeOptions());

        ~LanderAlignNode() override = default;

        void filterNormals();

        private:
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr mCostMapPub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mPcSub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPCDebugPub;

        geometry_msgs::msg::Pose mFinalAngle;

        rclcpp_action::Server<LanderAlign>::SharedPtr action_server_;

        //the callback for handling new goals (this implementation just accepts all goals)
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const LanderAlign::Goal> goal);

        //the callback for dealing with cancellation (this implementation just tells the client that it accepted the cancellation)
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleLanderAlign> goal_handle);

        //the callback for accepting a new goal and processing it
        void handle_accepted(const std::shared_ptr<GoalHandleLanderAlign> goal_handle);

        //all further processing and updates are done in the execute method in the new thread
        void execute(const std::shared_ptr<GoalHandleLanderAlign> goal_handle);

        /*
        Using PCA to calculate plane:
        1. Initial filtering of points by removing any normal with z-component above some threshold (plane normals will be
                relatively flat)

        2. Filter out additional points that are some threshold above the median of all points (75th %ile?)
        3. Once filtered, calculate mean and subtract it off from all points
        4. Do PCA and find normal
        5. Keep doing to find better planes?
        */
    };
}
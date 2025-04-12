#include "lander_align.new.hpp"
#include "lie.hpp"
#include "mrover/srv/detail/align_lander__struct.hpp"
#include <Eigen/src/Eigenvalues/EigenSolver.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <cmath>
#include <parameter.hpp>
#include <Eigen/src/Core/Matrix.h>
#include <cstddef>

//LanderAlignActionServer member functions:
namespace mrover{

    using LanderAlign = action::LanderAlign;
    using GoalHandleLanderAlign = rclcpp_action::ServerGoalHandle<LanderAlign>;
    
    LanderAlignNode::LanderAlignNode(const rclcpp::NodeOptions& options) 
        : Node("lander_align", options) {

        rclcpp::Service<mrover::srv::AlignLander>::SharedPtr start =
            create_service<mrover::srv::AlignLander>("align_lander", &startCallBack);

        mSensorSub = create_subscription<sensor_msgs::msg::PointCloud2>("/zed/left/points", 1, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
            LanderAlignNode::pointCloudCallback(msg);
        });

        mLanderService = create_service<mrover::srv::AlignLander>("align_lander", [this](mrover::srv::AlignLander::Request::ConstSharedPtr request,
                                                                                                                mrover::srv::AlignLander::Response::SharedPtr response){
           LanderAlignNode::startAlignCallback(request, response); 
        });
    }

    void LanderAlignNode::pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) {
        if(!mRunCallback){ return; }

        // Filter normals
        SE3f mapToCamera = SE3Conversions::fromTfTree(mTfBuffer, "map", "zed_left_camera_frame");
        auto* points = reinterpret_cast<Point const*>(msg->data.data());
        int numRows = 0;

        // Count how many rows the point matrix will have
        R3f unitZ(0.0, 0.0, 1.0);
        R3f zNormalInCamera = mapToCamera.act(unitZ).normalized();
        for(size_t i = 0; i < msg->height * msg->width; i++){
            Point const& point = points[i];

            if(
                std::isfinite(point.x) &&
                std::isfinite(point.y) &&
                std::isfinite(point.z) &&
                std::isfinite(point.normal_x) &&
                std::isfinite(point.normal_y) && 
                std::isfinite(point.normal_z) &&
                abs(zNormalInCamera.dot(R3f{point.normal_x, point.normal_y, point.normal_z})) < Z_NORM_MAX &&
                R3d(point.x, point.y, point.z).norm() < FAR_CLIP &&
                R3d(point.x, point.y, point.z).norm() > NEAR_CLIP
            ){
             numRows++;   
            }
        }

        // Fill the matrix of points
        Eigen::MatrixXd rows(numRows, 3);
        int rowNum = 0;
        for(size_t p = 0; p < msg->height * msg->width; p++){
            Point const& point = points[p];

            if(
                std::isfinite(point.x) &&
                std::isfinite(point.y) &&
                std::isfinite(point.z) &&
                std::isfinite(point.normal_x) &&
                std::isfinite(point.normal_y) && 
                std::isfinite(point.normal_z) &&
                abs(zNormalInCamera.dot(R3f{point.normal_x, point.normal_y, point.normal_z})) < Z_NORM_MAX &&
                R3d(point.x, point.y, point.z).norm() < FAR_CLIP &&
                R3d(point.x, point.y, point.z).norm() > NEAR_CLIP
            ){

                rows(rowNum, 0) = point.x;
                rows(rowNum, 1) = point.y;
                rows(rowNum, 2) = point.z;
                rowNum++;
            }
        }

        // Subtract off mean of each column from each column
        Eigen::Vector3d means = rows.colwise().mean();
        rows = rows.rowwise() - means.transpose();

        // Calculate covariance matrix and eigenvalues
        Eigen::Matrix3d covariance = rows.transpose() * rows;
        Eigen::EigenSolver<Eigen::Matrix3d> eigenvals(covariance);

        // Find the min eigenvalue (the associated eigenvector will be the plane normal)
        Eigen::Vector3d values = eigenvals.eigenvalues().real();
        Eigen::Matrix3d vecs = eigenvals.eigenvectors().real();

        int min = 0;
        for(int i = 0; i < 3; i++){
            if(abs(values[min]) > abs(values[i])){ min = i; }
        }

        // Get plane normal
        auto planeNormal = vecs.col(min);

        // publish normal
        SE3d plane{means, Eigen::Quaterniond::Identity()};
        SE3d normal{means + planeNormal, Eigen::Quaterniond::Identity()};
        SE3Conversions::pushToTfTree(*mTfBroadcaster, "lander_plane", "zed_left_camera_frame", plane, get_clock()->now());
        SE3Conversions::pushToTfTree(*mTfBroadcaster, "lander_normal", "zed_left_camera_frame", normal, get_clock()->now());
        mRunCallback = false;
    }

    auto LanderAlignNode::startAlignCallback(mrover::srv::AlignLander::Request::ConstSharedPtr& req, mrover::srv::AlignLander::Response::SharedPtr& res) -> void{
        mRunCallback = true;
        res->success = true;
    }

}

int main(void) {
    return 0;
}
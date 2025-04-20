#include "lander_align.new.hpp"
#include "lie.hpp"
#include "mrover/srv/detail/align_lander__struct.hpp"
#include <Eigen/src/Eigenvalues/EigenSolver.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <cmath>
#include <parameter.hpp>
#include <Eigen/src/Core/Matrix.h>
#include <cstddef>
#include <rclcpp/logging.hpp>

//LanderAlignActionServer member functions:
namespace mrover{    
    LanderAlignNode::LanderAlignNode(const rclcpp::NodeOptions& options) 
        : Node("lander_align", options) {

        mPcSub = create_subscription<sensor_msgs::msg::PointCloud2>("/zed/left/points", 1, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) {
            LanderAlignNode::pointCloudCallback(msg);
        });

        mLanderService = create_service<mrover::srv::AlignLander>("align_lander", [this](mrover::srv::AlignLander::Request::ConstSharedPtr request,
                                                                                                                mrover::srv::AlignLander::Response::SharedPtr response){
           LanderAlignNode::startAlignCallback(request, response); 
        });

        mDebugPub = create_publisher<sensor_msgs::msg::PointCloud2> ("landerAlign/debugPC", 1);
    }

    void LanderAlignNode::pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) {
        if(!mRunCallback){ return; }

        // Filter normals
        SE3d cameraToMap = SE3Conversions::fromTfTree(mTfBuffer, "zed_left_camera_frame", "map");
        auto* points = reinterpret_cast<Point const*>(msg->data.data());
        int numRows = 0;

        // Count how many rows the point matrix will have
        R3d unitZ(0.0, 0.0, 1.0);
        for(size_t i = 0; i < msg->height * msg->width; i++){
            Point const& point = points[i];
            R3d normalInMap = cameraToMap.rotation() * R3d{point.normal_x, point.normal_y, point.normal_z};
            normalInMap.normalize();

            if(
                std::isfinite(point.x) &&
                std::isfinite(point.y) &&
                std::isfinite(point.z) &&
                std::isfinite(point.normal_x) &&
                std::isfinite(point.normal_y) && 
                std::isfinite(point.normal_z) &&
                abs(unitZ.dot(normalInMap)) < Z_NORM_MAX &&
                R3d(normalInMap.x(), normalInMap.y(), normalInMap.z()).norm() < FAR_CLIP &&
                R3d(normalInMap.x(), normalInMap.y(), normalInMap.z()).norm() > NEAR_CLIP
            ){
             numRows++;   
            }
        }

        if constexpr(DEBUG_PC) {
            mDebugPC.reserve(numRows);
        }

        // Fill the matrix of points
        Eigen::MatrixXd rows(numRows, 3);
        int rowNum = 0;
        for(size_t p = 0; p < msg->height * msg->width; p++){
            Point const& point = points[p];
            R3d pointInMap = cameraToMap.act(R3d{point.x, point.y, point.z});
            R3d normalInMap = cameraToMap.rotation() * R3d{point.normal_x, point.normal_y, point.normal_z};
            normalInMap.normalize();

            if(
                std::isfinite(point.x) &&
                std::isfinite(point.y) &&
                std::isfinite(point.z) &&
                std::isfinite(point.normal_x) &&
                std::isfinite(point.normal_y) && 
                std::isfinite(point.normal_z) &&
                abs(unitZ.dot(normalInMap)) < Z_NORM_MAX &&
                R3d(normalInMap.x(), normalInMap.y(), normalInMap.z()).norm() < FAR_CLIP &&
                R3d(normalInMap.x(), normalInMap.y(), normalInMap.z()).norm() > NEAR_CLIP
            ){

                rows(rowNum, 0) = pointInMap.x();
                rows(rowNum, 1) = pointInMap.y();
                rows(rowNum, 2) = pointInMap.z();
                rowNum++;

                if constexpr(DEBUG_PC) {
                    mDebugPC.push_back(point);
                }
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
        mRunCallback = true;

        if constexpr(DEBUG_PC) {
            uploadDebugPC();
        }
    }

    auto LanderAlignNode::startAlignCallback(mrover::srv::AlignLander::Request::ConstSharedPtr& req, mrover::srv::AlignLander::Response::SharedPtr& res) -> void{
        mRunCallback = req->is_start;
        mRunCallback = true;
        res->success = true;
    }

    void LanderAlignNode::uploadDebugPC() {
        // RCLCPP_INFO_STREAM(get_logger(), "UPLOADING DEBUG PC");
        auto debugPointCloudPtr = std::make_unique<sensor_msgs::msg::PointCloud2>();
        fillPointCloudMessageHeader(debugPointCloudPtr);
        debugPointCloudPtr->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        debugPointCloudPtr->is_dense = true;
        debugPointCloudPtr->height = 1;
        debugPointCloudPtr->width = mDebugPC.size();
        debugPointCloudPtr->header.stamp = get_clock()->now();
        debugPointCloudPtr->header.frame_id = "zed_left_camera_frame";
        debugPointCloudPtr->data.resize(mDebugPC.size() * sizeof(Point));

		std::memcpy(debugPointCloudPtr->data.data(), mDebugPC.data(), mDebugPC.size() * sizeof(Point));

    	mDebugPub->publish(std::move(debugPointCloudPtr));
    }

}

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::LanderAlignNode>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
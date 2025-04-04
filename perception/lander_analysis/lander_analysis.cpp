#include "lander_analysis.hpp"
#include <cstddef>

namespace mrover {
    LanderAnalysis::LanderAnalysis(rclcpp::NodeOptions const& options) : rclcpp::Node{NODE_NAME, options}{
        RCLCPP_INFO_STREAM(get_logger(), "Created Node");

        mPCDebugPub = create_publisher<sensor_msgs::msg::PointCloud2>("/lander_analysis/debug_pc", 1);

        mPcSub = create_subscription<sensor_msgs::msg::PointCloud2>("/zed/left/points", 1, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) {
            LanderAnalysis::pointCloudCallback(msg);
        });
    }

    auto LanderAnalysis::pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) -> void{
        RCLCPP_INFO_STREAM(get_logger(), "Running PC Callback");

        // find the means for the point cloud
        Point const* pts = reinterpret_cast<Point const*>(msg->data.data());

        static constexpr float PLANAR_Z_THRESHOLD = 0.5;
        static constexpr float FAR_CLIP = 10;
        static constexpr float NEAR_CLIP = 0.5;

        // calculate how many inliers there will be
        std::size_t numInliers = 0;
        for(std::size_t i = 0; i < msg->width * msg->height; ++i){
            Point const& point = pts[i];

            if  (
                    std::isfinite(point.x) && 
                    std::isfinite(point.y) && 
                    std::isfinite(point.z) &&
                    std::isfinite(point.normal_x) &&
                    std::isfinite(point.normal_y) &&
                    std::isfinite(point.normal_z) &&
                    point.normal_z < PLANAR_Z_THRESHOLD &&
                    R3d(point.x, point.y, point.z).norm() < FAR_CLIP &&
                    R3d(point.x, point.y, point.z).norm() > NEAR_CLIP
                ) { 
                ++numInliers;
            }
        }

        Eigen::MatrixXd points(numInliers, 3);
        
        std::size_t row = 0;
        for(std::size_t i = 0; i < msg->width * msg->height; ++i){
            Point const& point = pts[i];

            if  (
                    std::isfinite(point.x) && 
                    std::isfinite(point.y) && 
                    std::isfinite(point.z) &&
                    std::isfinite(point.normal_x) &&
                    std::isfinite(point.normal_y) &&
                    std::isfinite(point.normal_z) &&
                    point.normal_z < PLANAR_Z_THRESHOLD &&
                    R3d(point.x, point.y, point.z).norm() < FAR_CLIP &&
                    R3d(point.x, point.y, point.z).norm() > NEAR_CLIP
                ) { 
                points(static_cast<int>(row), 0) = point.x;
                points(static_cast<int>(row), 1) = point.y;
                points(static_cast<int>(row), 2) = point.z;
                ++row;
            }
        }

        // mean
        Eigen::Vector3d mean = points.colwise().mean();

        RCLCPP_INFO_STREAM(get_logger(), "means " << mean << " rows" << points.colwise().mean().rows() << " cols " << points.colwise().mean().cols());

        // find covariance matrix
        Eigen::MatrixXd normalized(numInliers, 3);
        normalized = points.rowwise() - mean.transpose();
        Eigen::Matrix3d covariance = (normalized.transpose() * normalized) / (numInliers - 1);

        // RCLCPP_INFO_STREAM(get_logger(), "Covariance " << covariance);

        // find the eigen vectors and eigen values
        Eigen::EigenSolver<Eigen::Matrix3d> solver(covariance);

        Eigen::Vector3d vals = solver.eigenvalues().real();
        Eigen::Matrix3d vecs = solver.eigenvectors().real();

        int minIndex = 0;
        for(int i = 1; i < 3; ++i){
            if(std::abs(vals[minIndex]) > std::abs(vals[i])){
                minIndex = i;
            }
        }
        Eigen::Vector3d planeNormal = vecs.col(minIndex);

        // publish
        SE3d landerPlane{mean, Eigen::Quaterniond::Identity()};   
        SE3d landerPlaneNormal{mean + planeNormal, Eigen::Quaterniond::Identity()};        
        SE3Conversions::pushToTfTree(*mTfBroadcaster, "lander_plane", "zed_left_camera_frame", landerPlane, get_clock()->now());
        SE3Conversions::pushToTfTree(*mTfBroadcaster, "lander_plane_2", "zed_left_camera_frame", landerPlaneNormal, get_clock()->now());

        uploadPC(points);
    }
    void LanderAnalysis::uploadPC(Eigen::MatrixXd const& points) {
        auto debugPointCloudPtr = std::make_unique<sensor_msgs::msg::PointCloud2>();
        fillPointCloudMessageHeader(debugPointCloudPtr);
        debugPointCloudPtr->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        debugPointCloudPtr->is_dense = true;
        debugPointCloudPtr->height = 1;
        debugPointCloudPtr->width = points.rows();
        debugPointCloudPtr->header.stamp = get_clock()->now();
        debugPointCloudPtr->header.frame_id = "zed_left_camera_frame";
        debugPointCloudPtr->data.resize(points.rows() * sizeof(Point));

        // copy over the locations of each of the points
        Point *const pnt_ptr = reinterpret_cast<Point *const>(debugPointCloudPtr->data.data());
        for(std::size_t i = 0; i < static_cast<std::size_t>(points.rows()); ++i){
            pnt_ptr[i].x = points(i, 0);
            pnt_ptr[i].y = points(i, 1);
            pnt_ptr[i].z = points(i, 2);
        }

		

    	mPCDebugPub->publish(std::move(debugPointCloudPtr));
    }
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::LanderAnalysis);

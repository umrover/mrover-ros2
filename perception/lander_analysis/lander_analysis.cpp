#include "lander_analysis.hpp"

namespace mrover {
    LanderAnalysis::LanderAnalysis(rclcpp::NodeOptions const& options) : rclcpp::Node{NODE_NAME, options}{
        RCLCPP_INFO_STREAM(get_logger(), "Created Node");

        mPcSub = create_subscription<sensor_msgs::msg::PointCloud2>("/zed/left/points", 1, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) {
            LanderAnalysis::pointCloudCallback(msg);
        });
    }

    auto LanderAnalysis::pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) -> void{
        RCLCPP_INFO_STREAM(get_logger(), "Running PC Callback");

        // find the means for the point cloud
        Point const* pts = reinterpret_cast<Point const*>(msg->data.data());

        static constexpr float PLANAR_Z_THRESHOLD = 0.5;

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
                    point.normal_z < PLANAR_Z_THRESHOLD
                ) { 
                ++numInliers;
            }
        }

        Eigen::MatrixXf points(numInliers, 3);

        
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
                    point.normal_z < PLANAR_Z_THRESHOLD
                ) { 
                points(static_cast<int>(row), 0) = point.x;
                points(static_cast<int>(row), 1) = point.y;
                points(static_cast<int>(row), 2) = point.z;
            }
        }

        // mean
        Eigen::Vector3f mean = points.colwise().mean();

        // find covariance matrix
        Eigen::MatrixXf normalized(numInliers, 3);
        normalized = points.rowwise() - mean.transpose();
        Eigen::Matrix3f covariance = (normalized.transpose() * normalized) / (numInliers - 1);

        RCLCPP_INFO_STREAM(get_logger(), "Covariance " << covariance);

        // find the eigen vectors and eigen values
        Eigen::EigenSolver<Eigen::Matrix3f> solver(covariance);

        Eigen::Vector3f vals = solver.eigenvalues().real();
        Eigen::Matrix3f vecs = solver.eigenvectors().real();

        RCLCPP_INFO_STREAM(get_logger(), "Vals " << vals << " Vecs " << vecs);
    }
};

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::LanderAnalysis);

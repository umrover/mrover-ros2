#include "cost_map.hpp"

namespace mrover {

    constexpr static double IMU_WATCHDOG_TIMEOUT = 0.1;

    auto remap(double x, double inMin, double inMax, double outMin, double outMax) -> double {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    auto square(auto x) -> auto { return x * x; }

    auto mapToGrid(Eigen::Vector3f const& positionInMap, nav_msgs::msg::OccupancyGrid const& grid) -> int {
        Eigen::Vector2f origin{grid.info.origin.position.x, grid.info.origin.position.y};
        Eigen::Vector2f gridFloat = (positionInMap.head<2>() - origin) / grid.info.resolution;
        int gridX = std::floor(gridFloat.x());
        int gridY = std::floor(gridFloat.y());
        return gridY * static_cast<int>(grid.info.width) + gridX;
    }

    auto CostMapNode::pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) -> void {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        // if (!mLastImuTime || ros::Time::now() - mLastImuTime.value() > ros::Duration{IMU_WATCHDOG_TIMEOUT}) return;

        try {
            SE3f cameraToMap = SE3Conversions::fromTfTree(mTfBuffer, "zed_left_camera_frame", "map").cast<float>();

            struct BinEntry {
                R3f pointInCamera;
                R3f pointInMap;
            };
            using Bin = std::vector<BinEntry>;

            std::vector<Bin> bins;
            bins.resize(mGlobalGridMsg.data.size());

            auto* points = reinterpret_cast<Point const*>(msg->data.data());
            for (std::size_t r = 0; r < msg->height; r += mDownSamplingFactor) {
                for (std::size_t c = 0; c < msg->width; c += mDownSamplingFactor) {
                    Point const& point = points[r * msg->width + c];
                    R3f pointInCamera{point.x, point.y, point.z};

                    // Points with no stereo correspondence are NaN's, so ignore them
                    if (pointInCamera.hasNaN()) continue;

                    if (double distanceSquared = pointInCamera.squaredNorm();
                        distanceSquared < square(mNearClip) || distanceSquared > square(mFarClip)) continue;

                    R3f pointInMap = cameraToMap.act(pointInCamera);

                    int index = mapToGrid(pointInMap, mGlobalGridMsg);
                    if (index < 0 || index >= static_cast<int>(mGlobalGridMsg.data.size())) continue;

                    bins[index].emplace_back(BinEntry{pointInCamera, pointInMap});
                }
            }

            for (std::size_t i = 0; i < mGlobalGridMsg.data.size(); ++i) {
                Bin& bin = bins[i];
                if (bin.size() < 16) continue;

                std::size_t pointsHigh = std::ranges::count_if(bin, [this](BinEntry const& entry) {
                    return entry.pointInCamera.z() > mZThreshold;
                });
                double percent = static_cast<double>(pointsHigh) / static_cast<double>(bin.size());

                std::int8_t cost = percent > mZPercent ? OCCUPIED_COST : FREE_COST;

                // Update cell with EWMA acting as a low-pass filter
                auto& cell = mGlobalGridMsg.data[i];
                cell = static_cast<std::int8_t>(mAlpha * cost + (1 - mAlpha) * cell);
            }

            

            nav_msgs::msg::OccupancyGrid postProcessed = mGlobalGridMsg;
            std::ptrdiff_t width = postProcessed.info.width;

            std::array<std::ptrdiff_t, 11> dis
                                        {0, 
                                        -1, 
                                        +1, 
                                        -width, 
                                        -1 -width, 
                                        +1 -width, 
                                        +width, 
                                        -1 +width, 
                                        +1 +width};

            for (std::ptrdiff_t &di : dis) {
                mGlobalGridMsg.data[mapToGrid(cameraToMap.translation(), mGlobalGridMsg)+di] = FREE_COST;
            }

            for(std::size_t di_n = 0; di_n < 2; di_n++) {
                auto temp = postProcessed;
                for (std::size_t i = 0; i < postProcessed.data.size(); ++i) {
                    // If the current cell has any cost, then give it and all its neighbors high cost
                    std::size_t j;
                    if (temp.data[i] > FREE_COST) {
                        for (std::ptrdiff_t &di : dis) {
                            j = i + di;
                            if ((j / width >= 0 && j / width < std::size_t(postProcessed.info.height)) &&
                                (j % width >= 0 && j % width < std::size_t(width))) {
                                    postProcessed.data[j] = OCCUPIED_COST;
                                }
                        }
                    }
                }
            }
            mCostMapPub->publish(postProcessed);
        } catch (tf2::TransformException const& e) {
            RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, std::format("TF tree error processing point cloud: {}", e.what()));
        }
    }

    auto CostMapNode::moveCostMapCallback(mrover::srv::MoveCostMap::Request::ConstSharedPtr& req, mrover::srv::MoveCostMap::Response::SharedPtr& res) -> void {
        RCLCPP_INFO_STREAM(get_logger(), "Incoming request: " + req->course);
        SE3d centerInMap = SE3Conversions::fromTfTree(mTfBuffer, req->course, mMapFrame);
        std::ranges::fill(mGlobalGridMsg.data, UNKNOWN_COST);
        mGlobalGridMsg.info.origin.position.x = centerInMap.x() - mSize / 2;
        mGlobalGridMsg.info.origin.position.y = centerInMap.y() - mSize / 2;
        res->success = true;
        RCLCPP_INFO_STREAM(get_logger(), "Moved cost map");
    }

} // namespace mrover
#include "cost_map.hpp"
#include "lie.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <memory>
#include <stdexcept>

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

    auto CostMapNode::pointCloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr const& inputMsg) -> void {
        assert(inputMsg);
        assert(inputMsg->height > 0);
        assert(inputMsg->width > 0);
	
		// Choose whether we are using a noisy pointcloud or a regular pointcloud
		// TODO (john): change to shader
		sensor_msgs::msg::PointCloud2::UniquePtr noisyMsg = createNoisyPointCloud(inputMsg);

        if constexpr (useNoisyPointCloud) {
            mInliers.clear();
        }

		sensor_msgs::msg::PointCloud2::UniquePtr const& msg = [&]() -> sensor_msgs::msg::PointCloud2::UniquePtr const& {
			if constexpr (useNoisyPointCloud){
				return noisyMsg;
			}else{
				return inputMsg;
			}
		}();

        try {
            SE3f cameraToMap = SE3Conversions::fromTfTree(mTfBuffer, "zed_left_camera_frame", "map").cast<float>();
            SE3f roverSE3 = SE3Conversions::fromTfTree(mTfBuffer, "base_link", "map").cast<float>();

            struct BinEntry {
                R3f pointInCamera;
                R3f pointInMap;
                R3f normal;
            };

            using Bin = std::vector<BinEntry>;

            std::vector<Bin> bins;
            bins.resize(mGlobalGridMsg.data.size());

			// Clips the point cloud and fills bins the points
            auto* points = reinterpret_cast<Point const*>(msg->data.data());
            for (std::size_t r = 0; r < msg->height; r += mDownSamplingFactor) {
                for (std::size_t c = 0; c < msg->width; c += mDownSamplingFactor) {
                    Point const& point = points[r * msg->width + c];

                    R3f pointInCamera{point.x, point.y, point.z};
                    R3f normal{point.normal_x, point.normal_y, point.normal_z};

                    // Points with no stereo correspondence are NaN's, so ignore them
                    if (pointInCamera.hasNaN()) continue;

                    if (pointInCamera.y() > mRightClip &&
						pointInCamera.y() < mLeftClip &&
						pointInCamera.x() < mFarClip &&
						pointInCamera.x() > mNearClip){
						if constexpr (uploadDebugPointCloud){
							mInliers.push_back(point);
						}	
					}else{
						continue;
					}

                    R3f pointInMap = cameraToMap.act(pointInCamera);

                    int index = mapToGrid(pointInMap, mGlobalGridMsg);
                    if (index < 0 || index >= static_cast<int>(mGlobalGridMsg.data.size())) continue;

                    bins[index].emplace_back(BinEntry{pointInCamera, pointInMap, normal});
                }
            }
			
			// If we are using the debug point cloud publish it
			if constexpr (uploadDebugPointCloud){
				uploadPC();
			}

            for (std::size_t i = 0; i < mGlobalGridMsg.data.size(); ++i) {
				Bin& bin = bins[i];
                if (bin.size() < 16){
                    mGlobalGridMsg.data[i] = UNKNOWN_COST;
                }

                // USING ABSOLUTE HEIGHT DIFFERENCE BETWEEN POINT AND ROVER HEIGHT
                // std::size_t pointsHigh = std::ranges::count_if(bin, [this, &roverSE3](BinEntry const& entry) {
                //     return abs(entry.pointInMap.z() - (roverSE3.z())) > mZThreshold;
                // });
                // double percent = static_cast<double>(pointsHigh) / static_cast<double>(bin.size());

                // std::int8_t cost = percent > mZPercent ? OCCUPIED_COST : FREE_COST;

                // IMPLEMENT "AVERAGE" OF NORMALS IN BIN COMBINED WITH PROJECTION
                R3f avgNormal{};
                for(auto& point : bin){
                    avgNormal += point.normal;
                }

                avgNormal.normalize();
                std::int8_t cost = avgNormal.z() < mZThreshold ? OCCUPIED_COST : FREE_COST;

                // Update cell with EWMA acting as a low-pass filter
                auto& cell = mGlobalGridMsg.data[i];
                cell = static_cast<std::int8_t>(mAlpha * cost + (1 - mAlpha) * cell);
            }


			// Square Dilate operation
            nav_msgs::msg::OccupancyGrid postProcesed = mGlobalGridMsg;
            std::array<std::ptrdiff_t, 9> dis{0,
                                              -1, +1, -postProcesed.info.width, +postProcesed.info.width,
                                              -1 - postProcesed.info.width, +1 - postProcesed.info.width,
                                              -1 + postProcesed.info.width, +1 + postProcesed.info.width};
            for (std::size_t i = 0; i < mGlobalGridMsg.data.size(); ++i) {
                if (std::ranges::any_of(dis, [&](std::ptrdiff_t di) {
                        std::size_t j = i + di;
                        return j < mGlobalGridMsg.data.size() && mGlobalGridMsg.data[j] > FREE_COST;
                    })) postProcesed.data[i] = OCCUPIED_COST;
            }
            mCostMapPub->publish(postProcesed);
            if(!mCostMapPub){
                mCostMapPub->publish(postProcesed);
            }
        } catch (tf2::TransformException const& e) {
            RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, std::format("TF tree error processing point cloud: {}", e.what()));
        }
    }

	void CostMapNode::uploadPC() {
        auto debugPointCloudPtr = std::make_unique<sensor_msgs::msg::PointCloud2>();
        fillPointCloudMessageHeader(debugPointCloudPtr);
        debugPointCloudPtr->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        debugPointCloudPtr->is_dense = true;
        debugPointCloudPtr->height = 1;
        debugPointCloudPtr->width = mInliers.size();
        debugPointCloudPtr->header.stamp = get_clock()->now();
        debugPointCloudPtr->header.frame_id = "zed_left_camera_frame";
        debugPointCloudPtr->data.resize(mInliers.size() * sizeof(Point));

		std::memcpy(debugPointCloudPtr->data.data(), mInliers.data(), mInliers.size() * sizeof(Point));

    	mPCDebugPub->publish(std::move(debugPointCloudPtr));
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

    auto CostMapNode::processHeight(mrover::srv::MoveCostMap::Request::ConstSharedPtr& req, mrover::srv::MoveCostMap::Response::SharedPtr& res, std::vector<Bin> bins) -> void{

    }

} // namespace mrover

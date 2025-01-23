#include "cost_map.hpp"
#include "lie.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <format>
#include <memory>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <stdexcept>

namespace mrover {
    auto remap(double x, double inMin, double inMax, double outMin, double outMax) -> double {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    auto square(auto x) -> auto { return x * x; }

    auto mapToGrid(Eigen::Vector3f const& positionInMap, nav_msgs::msg::OccupancyGrid const& grid) -> int {
        Eigen::Vector2f origin{grid.info.origin.position.x, grid.info.origin.position.y};
        Eigen::Vector2f gridFloat = (positionInMap.head<2>() - origin) / grid.info.resolution;

        int gridX = std::floor(gridFloat.x());
        int gridY = std::floor(gridFloat.y());
	
		// Clip all of the points based on their relative location to the grid
		if(gridX >= static_cast<int>(grid.info.width) || gridX < 0 || gridY >= static_cast<int>(grid.info.height) || gridY < 0){
			return -1;
		}

        return gridY * static_cast<int>(grid.info.width) + gridX;
    }

    auto CostMapNode::pointCloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr const& inputMsg) -> void {
        assert(inputMsg);
        assert(inputMsg->height > 0);
        assert(inputMsg->width > 0);
	
		// Choose whether we are using a noisy pointcloud or a regular pointcloud
		// TODO (john): change to shader
		sensor_msgs::msg::PointCloud2::UniquePtr noisyMsg = createNoisyPointCloud(inputMsg);

		mInliers.clear();

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
                R3f normalInCamera;
                double height;
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

					if(index == -1){
						continue;
					}

                    bins[index].emplace_back(BinEntry{pointInCamera, pointInMap, normal, pointInMap.z() - roverSE3.z()});
                }
            }
			
			// If we are using the debug point cloud publish it
			if constexpr (uploadDebugPointCloud){
				uploadPC();
			}

            for (std::size_t i = 0; i < mGlobalGridMsg.data.size(); ++i) {
				Bin& bin = bins[i];
				auto& cell = mGlobalGridMsg.data[i];

				if (bin.size() < 16){
					cell = UNKNOWN_COST;
					continue;
				}

				std::size_t count = std::ranges::count_if(bin, [&](BinEntry const& entry){ return entry.normalInCamera.z() < mZThreshold; });

				RCLCPP_INFO_STREAM(get_logger(), "Count " << count);

				std::int8_t cost = count > 20 ? OCCUPIED_COST : FREE_COST;
				cell = cost;
			}

			RCLCPP_INFO_STREAM(get_logger(), "\n");

			// Square Dilate operation
            nav_msgs::msg::OccupancyGrid postProcesed = mGlobalGridMsg;
            std::array<std::ptrdiff_t, 9> disBottomLeft{0,
												  +1, +static_cast<std::ptrdiff_t>(postProcesed.info.width),
												  +1 + static_cast<std::ptrdiff_t>(postProcesed.info.width)};

            std::array<std::ptrdiff_t, 9> disTopLeft{0,
												  +1, -static_cast<std::ptrdiff_t>(postProcesed.info.width),
												  +1 - static_cast<std::ptrdiff_t>(postProcesed.info.width)};

            std::array<std::ptrdiff_t, 9> disBottomRight{0,
												  -1, +static_cast<std::ptrdiff_t>(postProcesed.info.width),
												  -1 + static_cast<std::ptrdiff_t>(postProcesed.info.width)};

            std::array<std::ptrdiff_t, 9> disTopRight{0,
												  -1, -static_cast<std::ptrdiff_t>(postProcesed.info.width),
												  -1 - static_cast<std::ptrdiff_t>(postProcesed.info.width)};

            std::array<std::ptrdiff_t, 9> disLeft{0,
												  +1, -static_cast<std::ptrdiff_t>(postProcesed.info.width), +static_cast<std::ptrdiff_t>(postProcesed.info.width),
												  +1 - static_cast<std::ptrdiff_t>(postProcesed.info.width),
												  +1 + static_cast<std::ptrdiff_t>(postProcesed.info.width)};

            std::array<std::ptrdiff_t, 9> disBottom{0,
												  -1, +1, +static_cast<std::ptrdiff_t>(postProcesed.info.width),
												  -1 + static_cast<std::ptrdiff_t>(postProcesed.info.width), +1 + static_cast<std::ptrdiff_t>(postProcesed.info.width)};

            std::array<std::ptrdiff_t, 9> disRight{0,
												  -1, -static_cast<std::ptrdiff_t>(postProcesed.info.width), +static_cast<std::ptrdiff_t>(postProcesed.info.width),
												  -1 - static_cast<std::ptrdiff_t>(postProcesed.info.width),
												  -1 + static_cast<std::ptrdiff_t>(postProcesed.info.width)};

            std::array<std::ptrdiff_t, 9> disTop{0,
												  -1, +1, -static_cast<std::ptrdiff_t>(postProcesed.info.width),
												  -1 - static_cast<std::ptrdiff_t>(postProcesed.info.width), +1 - static_cast<std::ptrdiff_t>(postProcesed.info.width)};

            std::array<std::ptrdiff_t, 9> disCenter{0,
												  -1, +1, -static_cast<std::ptrdiff_t>(postProcesed.info.width), +static_cast<std::ptrdiff_t>(postProcesed.info.width),
												  -1 - static_cast<std::ptrdiff_t>(postProcesed.info.width), +1 - static_cast<std::ptrdiff_t>(postProcesed.info.width),
												  -1 + static_cast<std::ptrdiff_t>(postProcesed.info.width), +1 + static_cast<std::ptrdiff_t>(postProcesed.info.width)};



            for (std::int64_t i = 0, msgSize = static_cast<std::int64_t>(mGlobalGridMsg.data.size()); i < msgSize; ++i) {
				// Bounds Check on the dilation index
				std::int64_t gridX = i % mGlobalGridMsg.info.width;
				std::int64_t gridY = i / mGlobalGridMsg.info.height;

				// TODO: This is kinda cooked
				if(gridX == 0 && gridY == 0){
					if (std::ranges::any_of(disBottomLeft, [&](std::ptrdiff_t di) {
							std::int64_t j = i + di;
							return j >= 0 && j < msgSize && mGlobalGridMsg.data[j] > FREE_COST;
						})){
						postProcesed.data[i] = OCCUPIED_COST;
					} 
					RCLCPP_INFO_STREAM(get_logger(), "Bottom Left");
				}else if(gridX == 0 && gridY == mGlobalGridMsg.info.height - 1){
					if (std::ranges::any_of(disTopLeft, [&](std::ptrdiff_t di) {
							std::int64_t j = i + di;
							return j >= 0 && j < msgSize && mGlobalGridMsg.data[j] > FREE_COST;
						})){
						postProcesed.data[i] = OCCUPIED_COST;
					} 
					RCLCPP_INFO_STREAM(get_logger(), "Top Left");
				}else if(gridX == mGlobalGridMsg.info.width - 1 && gridY == 0){
					if (std::ranges::any_of(disBottomRight, [&](std::ptrdiff_t di) {
							std::int64_t j = i + di;
							return j >= 0 && j < msgSize && mGlobalGridMsg.data[j] > FREE_COST;
						})){
						postProcesed.data[i] = OCCUPIED_COST;
					} 
					RCLCPP_INFO_STREAM(get_logger(), "Bottom Right");
				}else if(gridX == mGlobalGridMsg.info.width - 1 && gridY == mGlobalGridMsg.info.height - 1){
					if (std::ranges::any_of(disTopRight, [&](std::ptrdiff_t di) {
							std::int64_t j = i + di;
							return j >= 0 && j < msgSize && mGlobalGridMsg.data[j] > FREE_COST;
						})){
						postProcesed.data[i] = OCCUPIED_COST;
					} 
					RCLCPP_INFO_STREAM(get_logger(), "Top Right");
				}else if(gridX == 0){
					if (std::ranges::any_of(disLeft, [&](std::ptrdiff_t di) {
							std::int64_t j = i + di;
							return j >= 0 && j < msgSize && mGlobalGridMsg.data[j] > FREE_COST;
						})){
						postProcesed.data[i] = OCCUPIED_COST;
					} 
					RCLCPP_INFO_STREAM(get_logger(), "Left");
				}else if(gridY == 0){
					if (std::ranges::any_of(disBottom, [&](std::ptrdiff_t di) {
							std::int64_t j = i + di;
							return j >= 0 && j < msgSize && mGlobalGridMsg.data[j] > FREE_COST;
						})){
						postProcesed.data[i] = OCCUPIED_COST;
					} 
					RCLCPP_INFO_STREAM(get_logger(), "Bottom");
				}else if(gridX == mGlobalGridMsg.info.width - 1){
					if (std::ranges::any_of(disRight, [&](std::ptrdiff_t di) {
							std::int64_t j = i + di;
							return j >= 0 && j < msgSize && mGlobalGridMsg.data[j] > FREE_COST;
						})){
						postProcesed.data[i] = OCCUPIED_COST;
					} 
					RCLCPP_INFO_STREAM(get_logger(), "Right");
				}else if (gridY == mGlobalGridMsg.info.height - 1){
					if (std::ranges::any_of(disTop, [&](std::ptrdiff_t di) {
							std::int64_t j = i + di;
							return j >= 0 && j < msgSize && mGlobalGridMsg.data[j] > FREE_COST;
						})){
						postProcesed.data[i] = OCCUPIED_COST;
					} 
					RCLCPP_INFO_STREAM(get_logger(), "Top");
				}else{
					if (std::ranges::any_of(disCenter, [&](std::ptrdiff_t di) {
							std::int64_t j = i + di;
							if(i == 93){
								RCLCPP_INFO_STREAM(get_logger(), j << " " << di);
							}
							return j >= 0 && j < msgSize && mGlobalGridMsg.data[j] > FREE_COST;
						})){
						postProcesed.data[i] = OCCUPIED_COST;


						RCLCPP_INFO_STREAM(get_logger(), "Found");
					} 
					RCLCPP_INFO_STREAM(get_logger(), "Center");
				}
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
} // namespace mrover

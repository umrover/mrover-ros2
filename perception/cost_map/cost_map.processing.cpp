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
                R3f normal;
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

                if (bin.size() >= 16){
                    R3f avgNormal{};
					R3f avgPoseInCamera{};
                    for(auto& point : bin){
                        avgNormal += point.normal;
                        avgPoseInCamera += point.pointInCamera;
                    }

                    avgNormal.normalize();
					avgPoseInCamera.normalize();

					float stdDevZNormalInCamera{};
					float stdDevZPoseInCamera{};
                    for(auto& point : bin){
						stdDevZNormalInCamera += std::pow(point.normal.z() - avgNormal.z(), 2);
						stdDevZPoseInCamera += std::pow(point.pointInCamera.z() - avgPoseInCamera.z(), 2);
                    }

					stdDevZNormalInCamera /= static_cast<float>(bin.size());
					stdDevZPoseInCamera /= static_cast<float>(bin.size());

                    RCLCPP_INFO_STREAM(get_logger(), std::format("Normal Z avg {} std. dev. {}; Pose Z avg {} std. dev. {} num points {}", avgNormal.z(), stdDevZNormalInCamera, avgPoseInCamera.z(), stdDevZPoseInCamera, bin.size()));

					float prediction = x0 + x1 * avgNormal.z() + x2 * stdDevZNormalInCamera + x3 * avgPoseInCamera.z() + x4 * stdDevZPoseInCamera + x5 * static_cast<float>(bin.size());
					RCLCPP_INFO_STREAM(get_logger(), std::format("pred {}", prediction));

					// Update weights
					std::string s;
					while(s != "y" && s != "n"){
						std::cin >> s;
					}
					float label = (s == "y") ? 1 : -1;
					if(prediction * label <= 0){
						RCLCPP_INFO_STREAM(get_logger(), "Incorrect");
						x0 += label;
						x1 += label * avgNormal.z();
						x2 += label * stdDevZNormalInCamera;
						x3 += label * avgPoseInCamera.z();
						x4 += label * stdDevZPoseInCamera;
						x5 += label * static_cast<float>(bin.size());
					}else{
						RCLCPP_INFO_STREAM(get_logger(), "Correct");
					}

                    std::int8_t cost = avgNormal.z() < mZThreshold ? OCCUPIED_COST : FREE_COST;

                    // Update cell with EWMA acting as a low-pass filter
                    cell = static_cast<std::int8_t>(mAlpha * cost + (1 - mAlpha) * cell);
                }else{
					cell = UNKNOWN_COST;
				}
            }

			// Square Dilate operation
            nav_msgs::msg::OccupancyGrid postProcesed = mGlobalGridMsg;
            std::array<std::ptrdiff_t, 9> disBottomLeft{0,
												  +1, +postProcesed.info.width,
												  +1 + postProcesed.info.width};

            std::array<std::ptrdiff_t, 9> disTopLeft{0,
												  +1, -postProcesed.info.width,
												  +1 - postProcesed.info.width};

            std::array<std::ptrdiff_t, 9> disBottomRight{0,
												  -1, +postProcesed.info.width,
												  -1 + postProcesed.info.width};

            std::array<std::ptrdiff_t, 9> disTopRight{0,
												  -1, -postProcesed.info.width,
												  -1 - postProcesed.info.width};

            std::array<std::ptrdiff_t, 9> disLeft{0,
												  +1, -postProcesed.info.width, +postProcesed.info.width,
												  +1 - postProcesed.info.width,
												  +1 + postProcesed.info.width};

            std::array<std::ptrdiff_t, 9> disBottom{0,
												  -1, +1, +postProcesed.info.width,
												  -1 + postProcesed.info.width, +1 + postProcesed.info.width};

            std::array<std::ptrdiff_t, 9> disRight{0,
												  -1, -postProcesed.info.width, +postProcesed.info.width,
												  -1 - postProcesed.info.width,
												  -1 + postProcesed.info.width};

            std::array<std::ptrdiff_t, 9> disTop{0,
												  -1, +1, -postProcesed.info.width,
												  -1 - postProcesed.info.width, +1 - postProcesed.info.width};

            std::array<std::ptrdiff_t, 9> disCenter{0,
												  -1, +1, -postProcesed.info.width, +postProcesed.info.width,
												  -1 - postProcesed.info.width, +1 - postProcesed.info.width,
												  -1 + postProcesed.info.width, +1 + postProcesed.info.width};



            for (std::int64_t i = 0, msgSize = static_cast<std::int64_t>(mGlobalGridMsg.data.size()); i < msgSize; ++i) {
				// Bounds Check on the dilation index
				std::int64_t gridX = i % mGlobalGridMsg.info.width;
				std::int64_t gridY = i / mGlobalGridMsg.info.height;

				// TODO: This is kinda cooked
				if(gridX == 0 && gridY == 0){
					if (std::ranges::any_of(disBottomLeft, [&](std::ptrdiff_t di) {
							std::int64_t j = i + di;
							return j >= 0 && j < msgSize && mGlobalGridMsg.data[j] > FREE_COST;
						})) postProcesed.data[i] = OCCUPIED_COST;
				}else if(gridX == 0 && gridY == mGlobalGridMsg.info.height - 1){
					if (std::ranges::any_of(disTopLeft, [&](std::ptrdiff_t di) {
							std::int64_t j = i + di;
							return j >= 0 && j < msgSize && mGlobalGridMsg.data[j] > FREE_COST;
						})) postProcesed.data[i] = OCCUPIED_COST;
				}else if(gridX == mGlobalGridMsg.info.width - 1 && gridY == 0){
					if (std::ranges::any_of(disBottomRight, [&](std::ptrdiff_t di) {
							std::int64_t j = i + di;
							return j >= 0 && j < msgSize && mGlobalGridMsg.data[j] > FREE_COST;
						})) postProcesed.data[i] = OCCUPIED_COST;
				}else if(gridX == mGlobalGridMsg.info.width - 1 && gridY == mGlobalGridMsg.info.height - 1){
					if (std::ranges::any_of(disTopRight, [&](std::ptrdiff_t di) {
							std::int64_t j = i + di;
							return j >= 0 && j < msgSize && mGlobalGridMsg.data[j] > FREE_COST;
						})) postProcesed.data[i] = OCCUPIED_COST;
				}else if(gridX == 0){
					if (std::ranges::any_of(disLeft, [&](std::ptrdiff_t di) {
							std::int64_t j = i + di;
							return j >= 0 && j < msgSize && mGlobalGridMsg.data[j] > FREE_COST;
						})) postProcesed.data[i] = OCCUPIED_COST;
				}else if(gridY == 0){
					if (std::ranges::any_of(disBottom, [&](std::ptrdiff_t di) {
							std::int64_t j = i + di;
							return j >= 0 && j < msgSize && mGlobalGridMsg.data[j] > FREE_COST;
						})) postProcesed.data[i] = OCCUPIED_COST;
				}else if(gridX == mGlobalGridMsg.info.width - 1){
					if (std::ranges::any_of(disRight, [&](std::ptrdiff_t di) {
							std::int64_t j = i + di;
							return j >= 0 && j < msgSize && mGlobalGridMsg.data[j] > FREE_COST;
						})) postProcesed.data[i] = OCCUPIED_COST;
				}else if (gridY == mGlobalGridMsg.info.height - 1){
					if (std::ranges::any_of(disTop, [&](std::ptrdiff_t di) {
							std::int64_t j = i + di;
							return j >= 0 && j < msgSize && mGlobalGridMsg.data[j] > FREE_COST;
						})) postProcesed.data[i] = OCCUPIED_COST;
				}else{
					if (std::ranges::any_of(disCenter, [&](std::ptrdiff_t di) {
							std::int64_t j = i + di;
							return j >= 0 && j < msgSize && mGlobalGridMsg.data[j] > FREE_COST;
						})) postProcesed.data[i] = OCCUPIED_COST;
				}
            }

			/*
			for(size_t i = 0; i < mGlobalGridMsg.data.size(); ++i){
                RCLCPP_INFO_STREAM(get_logger(), std::format("{}: Post {} Pre {} Size {}", i, postProcesed.data[i], mGlobalGridMsg.data[i], bins[i].size()));
				SE3Conversions::pushToTfTree(mTfBroadcaster, std::format("bin{}", i), "map", SE3d{R3d{mGlobalGridMsg.info.origin.position.x + mResolution * (i % mGlobalGridMsg.info.width), mGlobalGridMsg.info.origin.position.y + mResolution * (i / mGlobalGridMsg.info.height), 1}, SO3d::Identity()}, get_clock()->now());
			}
			*/

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

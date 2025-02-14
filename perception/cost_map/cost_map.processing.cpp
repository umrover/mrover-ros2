#include "cost_map.hpp"
#include <algorithm>
#include <rclcpp/logging.hpp>

namespace mrover {
    auto remap(double x, double inMin, double inMax, double outMin, double outMax) -> double {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    auto square(auto x) -> auto { return x * x; }

    auto mapToGrid(Eigen::Vector3f const& positionInMap, nav_msgs::msg::OccupancyGrid const& grid) -> int {
        Eigen::Vector2f origin{grid.info.origin.position.x, grid.info.origin.position.y};
        Eigen::Vector2f gridFloat = (positionInMap.head<2>() - origin);

        // checking for out of bounds points
        // TODO: replace the width [or height] * resolution with mSize
        if(gridFloat.x() < 0 || gridFloat.x() > static_cast<float>(grid.info.width) * grid.info.resolution || 
            gridFloat.y() < 0 || gridFloat.y() > static_cast<float>(grid.info.height) * grid.info.resolution) return -1;        

        gridFloat /= grid.info.resolution;
        int gridX = std::floor(gridFloat.x());
        int gridY = std::floor(gridFloat.y());
        return gridY * static_cast<int>(grid.info.width) + gridX;
    }

    auto CostMapNode::pointCloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr const& inputMsg) -> void {
        assert(inputMsg);
        assert(inputMsg->height > 0);
        assert(inputMsg->width > 0);

        // RCLCPP_INFO_STREAM(get_logger(), "Pointer is " << inputMsg.get());
	
		// Choose whether we are using a noisy pointcloud or a regular pointcloud
		// TODO (john): change to shader
		//sensor_msgs::msg::PointCloud2::UniquePtr noisyMsg = createNoisyPointCloud(inputMsg);

		mInliers.clear();

		sensor_msgs::msg::PointCloud2::UniquePtr const& msg = inputMsg;/*[&]() -> sensor_msgs::msg::PointCloud2::UniquePtr const& {
			if constexpr (useNoisyPointCloud){
				return noisyMsg;
			}else{
				return inputMsg;
			}
		}();*/

        try {
            SE3f cameraToMap = SE3Conversions::fromTfTree(mTfBuffer, "zed_left_camera_frame", "map").cast<float>();
            SE3f roverSE3 = SE3Conversions::fromTfTree(mTfBuffer, "base_link", "map").cast<float>();

            // TIMING DEBUG
            // RCLCPP_INFO_STREAM(get_logger(), inputMsg->header.stamp.sec);

            // struct BinEntry {
            //     R3f pointInCamera;
            //     R3f pointInMap;
            //     R3f normal;
            // };

            // struct Bin {
            //     std::vector<BinEntry> points;
            //     int32_t i;  // index (used for for_each)
            // };

            struct Bin {
                int high_pts;
                int total;
            };

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
                    if(normal.hasNaN()) continue;
                    

                    if (pointInCamera.y() > mRightClip &&
						pointInCamera.y() < mLeftClip &&
						pointInCamera.x() < mFarClip &&
						pointInCamera.x() > mNearClip &&
                        pointInCamera.z() < mTopClip){
						if constexpr (uploadDebugPointCloud){
							mInliers.push_back(point);
						}	
					}else{
						continue;
					}

                    R3f pointInMap = cameraToMap.act(pointInCamera);

                    int index = mapToGrid(pointInMap, mGlobalGridMsg);
                    if (index < 0 || index >= static_cast<int>(mGlobalGridMsg.data.size())) continue;

                    
                    // TODO REPLACE this 
                    if(normal.z() <= mZThreshold) {
                        ++bins[index].high_pts;
                    }
                    ++bins[index].total;


                    // 1[index].points.emplace_back(BinEntry{pointInCamera, pointInMap, normal});
                }
            }
			
			// If we are using the debug point cloud publish it
			if constexpr (uploadDebugPointCloud){
				uploadPC();
			}

            for (std::size_t i = 0; i < mGlobalGridMsg.data.size(); ++i) {
				Bin& bin = bins[i];
                if (bin.total < 16){
                    continue;
                }

                // // Percentage Algorithm (acounts for angle changes (and outliers))
                // // Chose the percentage algorithm because it's less sensitive to angle changes (and outliers) and can accurately track
                // //     objects outside. Normal averaging too sensitive.
                // std::size_t pointsHigh = std::ranges::count_if(bin.points, [this, &roverSE3](BinEntry const& entry) {
                //     return (entry.normal.z()) <= mZThreshold;
                // });


                double percent = static_cast<double>(bin.high_pts) / static_cast<double>(bin.total);

                std::int8_t cost = percent > mZPercent ? OCCUPIED_COST : FREE_COST;

                // Update cell with EWMA acting as a low-pass filter
                auto& cell = mGlobalGridMsg.data[i];
                cell = static_cast<std::int8_t>(mAlpha * cost + (1 - mAlpha) * cell);
            }


			// Square Dilate operation
            nav_msgs::msg::OccupancyGrid postProcesed = mGlobalGridMsg;

            // Variable dilation for any width`
            std::array<CostMapNode::Coordinate,(2*dilation+1)*(2*dilation+1)> dis = diArray();

            // // RCLCPP_INFO_STREAM(get_logger(), std::format("Index: {}\tRow {}\tCol {}\tmWidth {}\tWidth2 {}", coordinateToIndex(dis[8]), dis[8].row, dis[8].col, mWidth, postProcesed.info.width));
            // // for (int row = 0; row < mWidth; ++row) {
            // //     for(int col = 0; col < mHeight; ++col) {
            // //         int oned_index = coordinateToIndex({row, col});
            // std::for_each(bins.begin(), bins.end(), [&](Bin& cell) {
            //     auto coord = indexToCoordinate(cell.i);
            //     // RCLCPP_INFO_STREAM(get_logger(), std::format("Testing Index: {}", oned_index));
            //     if(std::ranges::any_of(dis, [&](CostMapNode::Coordinate di) {
            //         // the coordinate of the cell we are checking + dis offset
            //         Coordinate dcoord = {coord.row + di.row, coord.col + di.col};
            //         if(dcoord.row < 0 || dcoord.row >= mHeight || dcoord.col < 0 || dcoord.col >= mWidth)
            //             return false;

            //         // RCLCPP_INFO_STREAM(get_logger(), std::format("Index: {}", coordinateToIndex(dcoord)));
            //         return mGlobalGridMsg.data[coordinateToIndex(dcoord)] > FREE_COST;
            //     })) postProcesed.data[cell.i] = OCCUPIED_COST;
            //     // }
            // });

            // TODO: consider running the dilation 2x instead of using a 5x5 "kernel"
            for (int row = 0; row < mWidth; ++row) {
                for(int col = 0; col < mHeight; ++col) {
                    int oned_index = coordinateToIndex({row, col});
                    // RCLCPP_INFO_STREAM(get_logger(), std::format("Testing Index: {}", oned_index));
                    if(std::ranges::any_of(dis, [&](CostMapNode::Coordinate di) {
                        // the coordinate of the cell we are checking + dis offset
                        Coordinate dcoord = {row + di.row, col + di.col};
                        if(dcoord.row < 0 || dcoord.row >= mHeight || dcoord.col < 0 || dcoord.col >= mWidth)
                            return false;

                        // RCLCPP_INFO_STREAM(get_logger(), std::format("Index: {}", coordinateToIndex(dcoord)));
                        return mGlobalGridMsg.data[coordinateToIndex(dcoord)] > FREE_COST;
                    })) postProcesed.data[oned_index] = OCCUPIED_COST;
                }
            }

            // }
            mCostMapPub->publish(postProcesed);
        } catch (tf2::TransformException const& e) {
            RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, std::format("TF tree error processing point cloud: {}", e.what()));
        }
    }

    // Resolved at compile time, returns dilation kernel
    constexpr auto CostMapNode::diArray()->std::array<CostMapNode::Coordinate,(2*dilation+1)*(2*dilation+1)>{
        std::array<CostMapNode::Coordinate, (2*dilation+1)*(2*dilation+1)> di;
        int pos = 0;

        for(int r = -dilation; r <= dilation; r++){
            for(int c = -dilation; c <= dilation; c++){
                di[pos] = {r, c};
                pos++;
            }
        }

        return di;
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

    auto CostMapNode::indexToCoordinate(const int index) -> CostMapNode::Coordinate {
        return {index / mWidth, index % mWidth};
    }

    auto CostMapNode::coordinateToIndex(const Coordinate c) -> int{
        return c.row * mWidth + c.col;
    }

} // namespace mrover

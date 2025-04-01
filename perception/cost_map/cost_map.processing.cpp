#include "cost_map.hpp"
#include "lie.hpp"
#include <cstdlib>
#include <limits>
#include <manif/impl/se3/SE3.h>

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

    auto CostMapNode::pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) -> void {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        // Push transforms b/w clip box and map frame to tf tree
        // SE3d rightBot(R3d{mRightClip, mNearClip, 0}, SO3d::Identity());
        // SE3d rightTop(R3d{mRightClip, mFarClip, 0}, SO3d::Identity());
        // SE3d leftBot(R3d{mLeftClip, mNearClip, 0}, SO3d::Identity());
        // SE3d leftTop(R3d{mLeftClip, mFarClip, 0}, SO3d::Identity());

        SE3d rightBot(R3d{mNearClip, mRightClip, 0}, SO3d::Identity());
        SE3d rightTop(R3d{mFarClip, mRightClip, 0}, SO3d::Identity());
        SE3d leftBot(R3d{mNearClip, mLeftClip, 0}, SO3d::Identity());
        SE3d leftTop(R3d{mFarClip, mLeftClip, 0}, SO3d::Identity());

        SE3Conversions::pushToTfTree(mTfBroadcaster, "bin_bot_left", "zed_left_camera_frame", leftBot, get_clock()->now());
        SE3Conversions::pushToTfTree(mTfBroadcaster, "bin_top_left", "zed_left_camera_frame", leftTop, get_clock()->now());
        SE3Conversions::pushToTfTree(mTfBroadcaster, "bin_bot_right", "zed_left_camera_frame", rightBot, get_clock()->now());
        SE3Conversions::pushToTfTree(mTfBroadcaster, "bin_top_right", "zed_left_camera_frame", rightTop, get_clock()->now());
                

        // RCLCPP_INFO_STREAM(get_logger(), "COST MAP MESSAGE PTR: " << msg.get());
        // RCLCPP_INFO_STREAM(get_logger(), "COST MAP MESSAGE TIME: " << msg->header.stamp.sec);

		mInliers.clear();

        try {
            SE3f cameraToMap = SE3Conversions::fromTfTree(mTfBuffer, "zed_left_camera_frame", "map").cast<float>();

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
                }
            }
			
			// If we are using the debug point cloud publish it
			if constexpr (uploadDebugPointCloud){
				uploadPC();
			}

            // Percentage Algorithm (acounts for angle changes (and outliers))
                // // Chose the percentage algorithm because it's less sensitive to angle changes (and outliers) and can accurately track
                // //     objects outside. Normal averaging too sensitive.

            // Boundary points
            leftBot = SE3Conversions::fromTfTree(mTfBuffer, "bin_bot_left", "map");
            leftTop = SE3Conversions::fromTfTree(mTfBuffer, "bin_top_left", "map");
            rightBot = SE3Conversions::fromTfTree(mTfBuffer, "bin_bot_right", "map");
            rightTop = SE3Conversions::fromTfTree(mTfBuffer, "bin_top_right", "map");

            double maxY = std::max(leftTop.y(), std::max(leftBot.y(), std::max(rightTop.y(), rightBot.y())));
            double minY = std::min(leftTop.y(), std::min(leftBot.y(), std::min(rightTop.y(), rightBot.y())));
            double minX = std::min(leftTop.x(), std::min(leftBot.x(), std::min(rightTop.x(), rightBot.x())));
            double maxX = std::max(leftTop.x(), std::max(leftBot.x(), std::max(rightTop.x(), rightBot.x())));
            for (std::size_t i = 0; i < mGlobalGridMsg.data.size(); ++i) {
				Bin& bin = bins[i];

                // If any part of the bin is outside of the clips, do not update it
                Coordinate binCoord = indexToCoordinate(i);
                double binX = mGlobalGridMsg.info.origin.position.x + (mResolution * binCoord.col)/2.0;
                double binY = mGlobalGridMsg.info.origin.position.y + (mResolution * binCoord.row)/2.0;

                if (bin.total < 16 || binX < minX || binX > maxX || binY < minY || binY > maxY){
                    // RCLCPP_INFO_STREAM(get_logger(), "DELETED BIN!");
                    continue;
                }

                RCLCPP_INFO_STREAM(get_logger(), "IN HERE!");

                // Calculate intersection point from bin center to each boundary (ray in the positive
                //         y-direction), we know the x-coordinate
                std::vector<double> ys(4);
                ys[0] = calcIntersection(leftTop.translation(), leftBot.translation(), binX);
                ys[1] = calcIntersection(leftBot.translation(), rightBot.translation(), binX);
                ys[2] = calcIntersection(rightBot.translation(), rightTop.translation(), binX);
                ys[3] = calcIntersection(rightTop.translation(), leftTop.translation(), binX);

                // count the number of valid intersections (ones that are in height bounds and in
                //        width bounds, and are above center Y)
                int validIntersections = 0;
                for(double y : ys){
                    // If the intersection point is not within the bin boundaries, or it is lower than
                    //         the bin, the intersection is not valid
                    if(y > maxY || y < minY || y < binY){ 
                        RCLCPP_INFO_STREAM(get_logger(), "INVALID Y!: " << y);
                        continue; 
                    }
                    validIntersections++;
                }

                // If the number of intersections are even, the bin is not valid
                if(validIntersections % 2 == 0){
                    RCLCPP_INFO_STREAM(get_logger(), "NUM INTERSECTIONS: " << validIntersections);
                    continue;
                }
                



                double percent = static_cast<double>(bin.high_pts) / static_cast<double>(bin.total);

                std::int8_t cost = percent > mZPercent ? OCCUPIED_COST : FREE_COST;

                // Update cell with EWMA acting as a low-pass filter
                auto& cell = mGlobalGridMsg.data[i];
                cell = static_cast<std::int8_t>(mAlpha * cost + (1 - mAlpha) * cell);
            }


			// Square Dilate operation
            nav_msgs::msg::OccupancyGrid postProcesed = mGlobalGridMsg;
            nav_msgs::msg::OccupancyGrid temp;

            // Variable dilation for any width`
            std::array<CostMapNode::Coordinate,(2*dilation+1)*(2*dilation+1)> dis = diArray();

            // std::array<CostMapNode::Coordinate, 5> crossDilation = {{{-1,0}}, 
            //                                                         {{0,-1}},
            //                                                         {{0,0}},
            //                                                         {{0,1}},
            //                                                         {{1,0}}};

            // Do one initial pass to generate post-process for 0 cell dilation (raw data); no actual dilation happening here
            //     Done so there is no grey scaling for 0 cell dilation
            for(auto& b : postProcesed.data){
                b = b > THRESHOLD_COST ? OCCUPIED_COST : (b == UNKNOWN_COST) ? UNKNOWN_COST : FREE_COST;
            }

            // Repeatedly apply 3x3 kernel (1 cell dilation)
            // TODO: running 2x dilation, but consider swapping which one is copying/being used to copy data to reduce time
            //       can do by swapping which one is being used at each times % 2 pass
            for(int times = 0; times < mDilateAmt; times++){
                temp = postProcesed;
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
                            return temp.data[coordinateToIndex(dcoord)] > FREE_COST;
                        })) postProcesed.data[oned_index] = OCCUPIED_COST;
                    }
                }
            }


            mCostMapPub->publish(postProcesed);
        } catch (tf2::TransformException const& e) {
            RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, std::format("TF tree error processing point cloud: {}", e.what()));
        }
    }

    // Resolved at compile time, returns dilation kernel
    constexpr auto CostMapNode::diArray()->std::array<CostMapNode::Coordinate,(2*dilation+1)*(2*dilation+1)>{
        std::array<CostMapNode::Coordinate, (2*dilation+1)*(2*dilation+1)> di{};
        int pos = 0;

        for(int r = -dilation; r <= dilation; r++){
            for(int c = -dilation; c <= dilation; c++){
                // Do not dilate the corners
                if((r == -dilation && c == -dilation) || 
                   (r == dilation && c == -dilation) || 
                   (r == -dilation && c == dilation) || 
                   (r == dilation && c == dilation)){ 
                    continue; 
                }

                di[pos] = {r, c};
                pos++;
            }
        }

        return di;
    }

    auto CostMapNode::calcIntersection(const R3d& startSeg, const R3d& endSeg, double binCenterX) -> double{
        R3d seg = endSeg - startSeg;
        if(seg.x() == 0){ return std::numeric_limits<double>::max(); }

        double t = (binCenterX - startSeg.x()) / seg.x();
        return seg.y() * t + startSeg.y();
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

    // REQ THE CENTER WE MUST PASS THE TOP LEFT
    auto CostMapNode::moveCostMapCallback(mrover::srv::MoveCostMap::Request::ConstSharedPtr& req, mrover::srv::MoveCostMap::Response::SharedPtr& res) -> void {
        RCLCPP_INFO_STREAM(get_logger(), "Incoming request: " + req->course);
        SE3d centerInMap = SE3Conversions::fromTfTree(mTfBuffer, req->course, mMapFrame);

        // Calculate new center for costmap, will be at nearest bin location. First round down to nearest int then use fmod to 
        //    add so that we effectively round to nearest multiple of mResolution
        int newLeftX = std::floor(centerInMap.x() - mSize/2.0);
        int newLeftY = std::floor(centerInMap.y() - mSize/2.0);
        double newTopLeftX = newLeftX + std::fmod(newLeftX, mResolution);
        double newTopLeftY = newLeftY + std::fmod(newLeftY, mResolution);

        double oldTopLeftX = mGlobalGridMsg.info.origin.position.x; // old top left X
        double oldTopLeftY = mGlobalGridMsg.info.origin.position.y; // old top left Y

        // One of the directions will cause overlap
        RCLCPP_INFO_STREAM(get_logger(), "Size " << mGlobalGridMsg.data.size());

        std::vector<int8_t> newGrid(mGlobalGridMsg.data.size(), UNKNOWN_COST);

        // Calculate delta row and delta col, subtract this transform to get from the new map back into the old one
        int deltaCol = static_cast<int>((oldTopLeftX - newTopLeftX) / mResolution);
        int deltaRow = static_cast<int>((oldTopLeftY - newTopLeftY) / mResolution);
        Coordinate transformCoord = {deltaRow, deltaCol}; 

        for(size_t i = 0; i < mGlobalGridMsg.data.size(); i++){
            // Calculate which bin we are at in (row,col) form
            Coordinate coord = indexToCoordinate(static_cast<int>(i));

            // transform to see which bin this is in the new map (possibly out of bounds)
            Coordinate newPos = coord - transformCoord;

            // Bounds check, if it is in bounds, means there is overlap
            if(newPos.row < 0 || newPos.col < 0 || newPos.row >= mHeight || newPos.col >= mWidth){ continue; }

            // Grab data from overlapping bin and put it in whatever that bin will become in new map
            newGrid[i] = mGlobalGridMsg.data[static_cast<size_t>(coordinateToIndex(newPos))];
        }

        std::swap(mGlobalGridMsg.data, newGrid);

        mGlobalGridMsg.info.origin.position.x = newTopLeftX;
        mGlobalGridMsg.info.origin.position.y = newTopLeftY;

        res->success = true;
        RCLCPP_INFO_STREAM(get_logger(), "Moved cost map");
    }

    auto CostMapNode::dilateCostMapCallback(mrover::srv::DilateCostMap::Request::ConstSharedPtr& req, mrover::srv::DilateCostMap::Response::SharedPtr& res) -> void{
        // TODO: consider floor vs ceil here, currently rounds down to nearest cell dilation amt
        mDilateAmt = static_cast<int>(floor(static_cast<double>(req->d_amt/mResolution)));
        res->success = true;
    }

    auto CostMapNode::indexToCoordinate(const int index) const -> CostMapNode::Coordinate {
        return {index / mWidth, index % mWidth};
    }

    auto CostMapNode::coordinateToIndex(const Coordinate c) const -> int{
        return c.row * mWidth + c.col;
    }

} // namespace mrover

#include "cost_map.hpp"

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

        SE3d rightBot(R3d{mNearClip, -mNearWidth, 0}, SO3d::Identity());
        SE3d rightTop(R3d{mFarClip, -mFarWidth, 0}, SO3d::Identity());
        SE3d leftBot(R3d{mNearClip, mNearWidth, 0}, SO3d::Identity());
        SE3d leftTop(R3d{mFarClip, mFarWidth, 0}, SO3d::Identity());

        SE3Conversions::pushToTfTree(mTfBroadcaster, "bin_bot_left", "zed_left_camera_frame", leftBot, get_clock()->now());
        SE3Conversions::pushToTfTree(mTfBroadcaster, "bin_top_left", "zed_left_camera_frame", leftTop, get_clock()->now());
        SE3Conversions::pushToTfTree(mTfBroadcaster, "bin_bot_right", "zed_left_camera_frame", rightBot, get_clock()->now());
        SE3Conversions::pushToTfTree(mTfBroadcaster, "bin_top_right", "zed_left_camera_frame", rightTop, get_clock()->now());

		mInliers.clear();

        try {
            SE3f cameraToMap = SE3Conversions::fromTfTree(mTfBuffer, "zed_left_camera_frame", "map").cast<float>();

            struct Bin {
                int high_pts;
                int total;
            };

            struct DebugBin{
                std::vector<Point> points;
            };

            std::vector<Bin> bins;
            std::vector<DebugBin> debugBins;
            debugBins.resize((mGlobalGridMsg.data.size()));
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

                    R3f pointInMap = cameraToMap.act(pointInCamera);

                    int index = mapToGrid(pointInMap, mGlobalGridMsg);
                    if (index < 0 || index >= static_cast<int>(mGlobalGridMsg.data.size())) continue;
                    

                    if constexpr(uploadDebugPointCloud){
                        // add point to corresponding bin
                        debugBins[index].points.push_back(point);
                    }
                    
                    // TODO REPLACE this 
                    if(normal.z() <= mZThreshold) {
                        ++bins[index].high_pts;
                    }
                    ++bins[index].total;
                }
            }
			
			

            // Percentage Algorithm (acounts for angle changes (and outliers))
                // // Chose the percentage algorithm because it's less sensitive to angle changes (and outliers) and can accurately track
                // //     objects outside. Normal averaging too sensitive.

            // Boundary points
            leftBot = SE3Conversions::fromTfTree(mTfBuffer, "bin_bot_left", "map");
            leftTop = SE3Conversions::fromTfTree(mTfBuffer, "bin_top_left", "map");
            rightBot = SE3Conversions::fromTfTree(mTfBuffer, "bin_bot_right", "map");
            rightTop = SE3Conversions::fromTfTree(mTfBuffer, "bin_top_right", "map");

            for (std::size_t i = 0; i < mGlobalGridMsg.data.size(); ++i) {
				Bin& bin = bins[i];
                
                // Skip if there are not enough points
                if (bin.total < 16){
                    continue;
                }

                // If any part of the bin is outside of the clips, do not update it
                Coordinate binCoord = indexToCoordinate(i);
                double binCenterX = mGlobalGridMsg.info.origin.position.x + (mResolution * binCoord.col)  + mResolution/2.0;
                double binCenterY = mGlobalGridMsg.info.origin.position.y + (mResolution * binCoord.row)  + mResolution/2.0;

                // Calculate intersection point from bin center to each boundary (ray in the positive
                //         y-direction), we know the x-coordinate
                std::int8_t numIntersection = 0;
                numIntersection += isRayIntersection(leftTop.translation(), leftBot.translation(), binCenterX, binCenterY);
                numIntersection += isRayIntersection(leftBot.translation(), rightBot.translation(), binCenterX, binCenterY);
                numIntersection += isRayIntersection(rightBot.translation(), rightTop.translation(), binCenterX, binCenterY);
                numIntersection += isRayIntersection(rightTop.translation(), leftTop.translation(), binCenterX, binCenterY);

                // if the number of intersections is 

                // If the number of intersections are even, the bin is not valid
                if(numIntersection % 2 == 0){
                    continue;
                }

                if constexpr (uploadDebugPointCloud){
                    for(Point& p : debugBins[i].points){
                        mInliers.push_back(p);
                    }
                }

                double percent = static_cast<double>(bin.high_pts) / static_cast<double>(bin.total);

                std::int8_t cost = percent > mZPercent ? OCCUPIED_COST : FREE_COST;

                // Update cell with EWMA acting as a low-pass filter
                auto& cell = mGlobalGridMsg.data[i];
                cell = static_cast<std::int8_t>(mAlpha * cost + (1 - mAlpha) * cell);
            }

            // If we are using the debug point cloud publish it
			if constexpr (uploadDebugPointCloud){
				uploadPC();
			}

            // Make a new occupancy grid that's mNumDivisions^2 times the size of the original
            nav_msgs::msg::OccupancyGrid postProcessed;
            postProcessed.info.resolution = mResolution/mNumDivisions;
            postProcessed.info.height = mGlobalGridMsg.info.height * mNumDivisions;
            postProcessed.info.width = mGlobalGridMsg.info.width * mNumDivisions;
            postProcessed.info.origin = mGlobalGridMsg.info.origin;
            postProcessed.header.frame_id = mMapFrame;
            postProcessed.data.resize(mNumDivisions * mNumDivisions * mGlobalGridMsg.data.size(), UNKNOWN_COST);

            // Fill each new cell in the new map with data from the old cell (each old cell will populate numDivisions^2 worth of data)
            for(int row = 0; row < mHeight; ++row){
                for(int col = 0; col < mWidth; ++col){
                    // Get current cell info, and make it either high, free, or unknown cost to remove grey scaling
                    int8_t cell = mGlobalGridMsg.data[coordinateToIndex({row, col})];
                    cell = cell > THRESHOLD_COST ? OCCUPIED_COST : (cell == UNKNOWN_COST) ? UNKNOWN_COST : FREE_COST;

                    // For each new cell, update it with the old cell info
                    for(int subRow = 0; subRow < mNumDivisions; ++subRow){
                        for(int subCol = 0; subCol < mNumDivisions; ++subCol){
                            int atRow = row * mNumDivisions + subRow;
                            int atCol = col * mNumDivisions + subCol;
                            postProcessed.data[atRow * postProcessed.info.width + atCol] = cell;
                        }
                    }
                }
            }


			// Square Dilate operation
            nav_msgs::msg::OccupancyGrid temp;

            // Repeatedly apply combination of kernels defined below to get a circular looking dilation
            static constexpr std::array<std::array<CostMapNode::Coordinate, 9>, 2> kernels = {
                std::array<CostMapNode::Coordinate, 9>{Coordinate{0,0}, {-1,0}, {0,0}, 
                 {0,-1}, {0,0}, {0,1},
                 {0,0}, {1,0}, {0,0}},

                {Coordinate{-1,-1}, {-1,0}, {-1,1},
                 {0,-1}, {0,0}, {0,1},
                 {1,-1}, {1,0}, {1,1}}
            };


            int width = mWidth * mNumDivisions;
            int height = mHeight * mNumDivisions;
            for(int times = 0; times < mDilateAmt; times++){
                for(auto& ker : kernels){
                    temp = postProcessed;
                    for (int row = 0; row < width; ++row) {
                        for(int col = 0; col < height; ++col) {
                            int oned_index = coordinateToIndex({row, col}, width);

                            if(temp.data[oned_index] == OCCUPIED_COST){ // if we are currently looking at a bin which is OCCUPIED COST skip so it doesnt get set to DILATION_COST
                                postProcessed.data[oned_index] = OCCUPIED_COST;
                                continue;
                            }

                            // RCLCPP_INFO_STREAM(get_logger(), std::format("Testing Index: {}", oned_index));
                            if(std::ranges::any_of(ker, [&](CostMapNode::Coordinate di) {
                                // the coordinate of the cell we are checking + dis offset
                                Coordinate dcoord = {row + di.row, col + di.col};
                                if(dcoord.row < 0 || dcoord.row >= height || dcoord.col < 0 || dcoord.col >= width)
                                    return false;
        
                                // RCLCPP_INFO_STREAM(get_logger(), std::format("Index: {}", coordinateToIndex(dcoord)));
                                return temp.data[coordinateToIndex(dcoord, width)] > FREE_COST;
                            })) postProcessed.data[oned_index] = DILATED_COST;
                        }
                    }
                }
            }


            mCostMapPub->publish(postProcessed);
        } catch (tf2::TransformException const& e) {
            RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, std::format("TF tree error processing point cloud: {}", e.what()));
        }
    }

    auto CostMapNode::isRayIntersection(const R3d& startSeg, const R3d& endSeg, double binCenterX, double binCenterY) -> std::int8_t{
        // if the point is not inside the bounds of the segment's x axis
        double lowerX = (startSeg.x() < endSeg.x()) ? startSeg.x() : endSeg.x();
        double higherX = (startSeg.x() < endSeg.x()) ? endSeg.x() : startSeg.x();
        if(binCenterX <= lowerX || binCenterX >= higherX){
            return 0;
        }

        R3d seg = endSeg - startSeg;

        // handle the case where the slope is undefined
        if(seg.x() == 0){
            return 0;
        }

        // calc the y value of the intersection point to see if it is in boundary
        double m = seg.y() / seg.x();
        double y = m * (binCenterX - startSeg.x()) + startSeg.y();

        double lowerY = (startSeg.y() < endSeg.y()) ? startSeg.y() : endSeg.y();
        double higherY = (startSeg.y() < endSeg.y()) ? endSeg.y() : startSeg.y();
        
        if(y <= lowerY || y >= higherY || y <= binCenterY){
            return 0;
        }

        return 1;
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

    auto CostMapNode::coordinateToIndex(const Coordinate c, int width) const -> int{
        return c.row * width + c.col;
    }

} // namespace mrover

#include "lander_align.hpp"
#include "mrover/action/detail/lander_align__struct.hpp"
#include <boost/smart_ptr/shared_ptr.hpp>
#include <memory>
#include <optional>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

namespace mrover {
    auto operator<<(std::ostream& ostream, RTRSTATE state) -> std::ostream& {
        return ostream << RTRSTRINGS[static_cast<std::size_t>(state)];
    }

    LanderAlign::LanderAlign() : rclcpp::Node(NODE_NAME) {
		// Get the values from ros 
        mZThreshold = .5;
        mXThreshold = .1;
        mPlaneOffsetScalar = 2.5;

        RCLCPP_INFO_STREAM(get_logger(), "KILL YOURSELF");
        mDebugVectorPub = create_publisher<geometry_msgs::msg::Vector3>("/lander_align/Pose", 1);
        mTwistPub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        mDebugPCPub = create_publisher<sensor_msgs::msg::PointCloud2>("/lander_align/debugPC", 1);

        mCloud = std::nullopt;
        mSubscriber = create_subscription<sensor_msgs::msg::PointCloud2>("/zed/left/points", 1, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg){
            LanderAlign::subscriberCallback(msg);
        });
    

		using GoalHandleLanderAlign = rclcpp_action::ServerGoalHandle<action::LanderAlign>;
        // explicit LanderAlign(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        // : Node("lander_align", options)
        // {   
        
        mActionServer = rclcpp_action::create_server<action::LanderAlign>(
            this,
            "LanderAlignAction",
            std::bind(&LanderAlign::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&LanderAlign::handle_cancel, this, std::placeholders::_1),
            std::bind(&LanderAlign::handle_accepted, this, std::placeholders::_1)
        );

        // mActionServer.value().start(); // This motherfucker

        auto paramSub = std::make_shared<rclcpp::ParameterEventHandler>(this);

        std::vector<ParameterWrapper> params{
                {"camera_frame", mCameraFrameId, "zed_left_camera_frame"},
                {"world_frame", mMapFrameId, "map"},
                {"ransac/distance_threshold", mDistanceThreshold}};

        ParameterWrapper::declareParameters(this, params);

        mPlaneLocationInWorldVector = std::make_optional<Eigen::Vector3d>(0, 0, 0);
        mNormalInWorldVector = std::make_optional<Eigen::Vector3d>(0, 0, 0);
        mLoopState = RTRSTATE::turn1;
    }

    void LanderAlign::subscriberCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg){
        if (mReadPointCloud) {
            mCloud = std::make_optional(msg);
        }
        // if(mCloud.has_value()){ return; }
        // else{  }
    }

    auto LanderAlign::ActionServerCallBack(std::shared_ptr<GoalHandleLanderAlign> const goal) -> void {
		RCLCPP_INFO_STREAM(get_logger(), "in action server callback");
        std::shared_ptr<action::LanderAlign_Result> result;

        //If we haven't yet defined the point cloud we are working with
		RCLCPP_INFO_STREAM(get_logger(), "before wait");
		
        while (!mCloud.has_value()) {
            mReadPointCloud = true;
        }

        if (mCloud.has_value()) {
            mReadPointCloud = false;
            RCLCPP_INFO_STREAM(get_logger(), "cloud has value");
            filterNormals(mCloud.value());
            ransac(1, 10, 100);
            RCLCPP_INFO_STREAM(get_logger(), "post ransac");
            try{
                if(mNormalInWorldVector.has_value()){
                    if(!createSpline(7, 0.75)){
                        rclcpp::shutdown(); 
                        return;
                    }
                    RCLCPP_INFO_STREAM(get_logger(), "spline created");    
                    publishSpline();
                    //calcMotionToo();
                }
            } catch(const tf2::LookupException& e){
                rclcpp::shutdown();
                return;
            }
        // }

        mPathPoints.clear();
        } else {
            RCLCPP_INFO_STREAM(get_logger(), "cloud not has value");
        }
        // RCLCPP_INFO_STREAM(get_logger(), mCloud.value());
        // if (mCloud.has_value()) {
        // RCLCPP_INFO_STREAM(get_logger(), "cloud has value");

        
    }

    //Returns angle (yaw) around the z axis
    auto LanderAlign::calcAngleWithWorldX(Eigen::Vector3d xHeading) -> double { //I want to be editing this variable so it should not be const or &
        xHeading.z() = 0;
        xHeading.normalize();

        Eigen::Vector3d xAxisWorld{1, 0, 0};
        double angle = std::acos(xHeading.dot(xAxisWorld));
        if (xHeading.y() >= 0) {
            return angle;
        } else {
            return 2 * std::numbers::pi - angle;
        }
    };

    void LanderAlign::calcMotionToo() {
        SE3d roverInWorld = SE3Conversions::fromTfTree(*mTfBuffer, "base_link", "map");
        
        // Inital State
        Eigen::Vector2d initState{roverInWorld.translation().x(), roverInWorld.translation().y()};

        for (const Vector5d& point : mPathPoints) {
            double K1 = .3;
            double K2 = 1.4;
            double K3 = 1;  

            // Grab the current target state from the spline
            Eigen::Vector3d tarState{point.coeff(0, 0), point.coeff(1, 0), point.coeff(2, 0)};

            // Publish the target position in the spline
            Eigen::Matrix3d rot;
            rot <<  1, 0, 0,
                    0, 1, 0,
                    0, 0, 1;
            SE3d temp = {{point.coeff(0,0), point.coeff(1, 0), 0}, SO3d{Eigen::Quaterniond{rot}.normalized()}};
            SE3Conversions::pushToTfTree(*mTfBroadcaster, "spline_point", mMapFrameId, temp, get_clock()->now());
            RCLCPP_INFO_STREAM(get_logger(), "Switching to target position: (x, y, theta): (" << tarState.x() << ", " << tarState.y() << ", " << tarState.z() << ")");

            double distanceToTarget = std::numeric_limits<double>::max();

            while (rclcpp::ok() && distanceToTarget > 0.7) {
                roverInWorld = SE3Conversions::fromTfTree(*mTfBuffer, "base_link", "map");
                Eigen::Vector3d xOrientation = roverInWorld.rotation().col(0); 
                double roverHeading = calcAngleWithWorldX(xOrientation);
                Eigen::Vector3d currState{roverInWorld.translation().x(), roverInWorld.translation().y(), roverHeading};

                Eigen::Vector2d distanceToTargetVector{tarState.x() - currState.x(), tarState.y() - currState.y()};
                distanceToTarget = distanceToTargetVector.norm();

                Eigen::Matrix3d rotation;
                rotation << std::cos(roverHeading),  std::sin(roverHeading), 0,
                            -std::sin(roverHeading), std::cos(roverHeading), 0,
                            0,                         0,                      1;
                
                Eigen::Vector3d errState = rotation * (currState - tarState); // maybe check error angle incase anything goes silly
                RCLCPP_INFO_STREAM(get_logger(), "err state " << errState.coeff(0,0) << ", " << errState.coeff(1,0) << ", " << errState.coeff(2,0));
                
                RCLCPP_INFO_STREAM(get_logger(), "Rover Heading " << roverHeading);

                RCLCPP_INFO_STREAM(get_logger(), "Term 1: " << (K2*point.coeff(3, 0)*errState.y())*pow(cos(errState.z()), 2));
                RCLCPP_INFO_STREAM(get_logger(), "Term 2: " << (K3*abs(point.coeff(3, 0))*tan(errState.z()))*pow(cos(errState.z()), 2));
                
				rclcpp::Rate r(30);
                r.sleep();
                double v = (point.coeff(3, 0) - K1 * abs(point.coeff(3, 0) * (errState.x() + errState.y() * tan(errState.z()))))/(cos(errState.z()));
                double omega = point.coeff(4, 0) - ((K2*point.coeff(3, 0)*errState.y() + K3*abs(point.coeff(3, 0))*tan(errState.z()))*pow(cos(errState.z()), 2));
                RCLCPP_INFO_STREAM(get_logger(), "v: " << v);
                RCLCPP_INFO_STREAM(get_logger(), "omega: " << omega);
                RCLCPP_INFO_STREAM(get_logger(), "tan: " << tan(errState.z()));
                
                driveTwist.angular.z = omega;
                driveTwist.linear.x = v;
                driveTwist.angular.z = 0;
                driveTwist.linear.x = 0;
                mTwistPub->publish(driveTwist);
            }        
        }

        // Final Turn Adjustment
        {
            //Locations
            Eigen::Vector3d rover_dir;

            //Threhsolds
            float const angular_thresh = 0.001;


            rclcpp::Rate rate(20); // ROS Rate at 20Hzn::Matrix3d roverToPlaneNorm;
            
            while (rclcpp::ok()) {                
                SE3d roverInWorld = SE3Conversions::fromTfTree(*mTfBuffer, "base_link", "map");
                Eigen::Vector3d roverPosInWorld{(roverInWorld.translation().x()), (roverInWorld.translation().y()), 0.0};

                Eigen::Vector3d roverToTargetForward = -mNormalInWorldVector.value();
                roverToTargetForward.normalize();

                Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
                Eigen::Vector3d left = up.cross(roverToTargetForward);

                Eigen::Matrix3d roverToTargetMat;
                roverToTargetMat.col(0) = roverToTargetForward;
                roverToTargetMat.col(1) = up;
                roverToTargetMat.col(2) = left;

                //SO3 Matrices for lie algebra
                SO3d roverToTargetSO3 = SE3Conversions::fromColumns(roverToTargetForward, left, up);
                SO3d roverSO3 = roverInWorld.asSO3();

                manif::SO3Tangentd SO3tan = roverToTargetSO3 - roverSO3; // 3 x 1 matrix of angular velocities (x,y,z)

                double angle_rate = mAngleP * SO3tan.z();
                angle_rate = (std::abs(angle_rate) > mAngleFloor) ? angle_rate : copysign(mAngleFloor, angle_rate);
                turnTwist.angular.z = angle_rate;
                turnTwist.linear.x = 0;

                if (std::abs(SO3tan.z()) < angular_thresh) {
                    break;
                }
                mTwistPub->publish(turnTwist);
            }
        }
        
        driveTwist.angular.z = 0;
        driveTwist.linear.x = 0;
        mTwistPub->publish(driveTwist);

        // rclcpp::shutdown(); // perchance a touch yuckycky
    }

    void LanderAlign::filterNormals(sensor_msgs::msg::PointCloud2::ConstSharedPtr const& cloud) {
        RCLCPP_INFO_STREAM(get_logger(), "in filter normals");
        mFilteredPoints.clear();
        
        // Pointer to the underlying point cloud data
        auto* cloudData = reinterpret_cast<Point const*>(cloud->data.data());
        // RCLCPP_INFO_STREAM(get_logger(),"this part");
        std::default_random_engine generator;
        std::uniform_int_distribution<int> pointDistribution(0, mLeastSamplingDistribution);

        // RCLCPP_INFO_STREAM(get_logger(),"this part2");

        // Loop over the entire PC
        for (auto point = cloudData; point < cloudData + (cloud->height * cloud->width); point += pointDistribution(generator)) {
            // Make sure all of the values are defined
            // RCLCPP_INFO_STREAM(get_logger(),"this part3");

            bool isPointInvalid = (!std::isfinite(point->x) || !std::isfinite(point->y) || !std::isfinite(point->z));
            if (!isPointInvalid && abs(point->normal_z) < mZThreshold && abs(point->normal_x) > mXThreshold) {
                    mFilteredPoints.push_back(point);
            }
            // RCLCPP_INFO_STREAM(get_logger(),"this part4");
        }

        RCLCPP_INFO_STREAM(get_logger(), "Filtered Points: " << mFilteredPoints.size());
    }

    void LanderAlign::uploadPC(int numInliers, double distanceThreshold) {
        // auto debugPointCloudPtr = boost::make_shared<sensor_msgs::PointCloud2>();
        RCLCPP_INFO_STREAM(get_logger(), "in uploadPC");
        
        auto debugPointCloudPtr = std::make_shared<sensor_msgs::msg::PointCloud2>();
        
        RCLCPP_INFO_STREAM(get_logger(), "pre fillpointcloudmessage");

        fillPointCloudMessageHeader(debugPointCloudPtr);
        RCLCPP_INFO_STREAM(get_logger(), "post fillpointcloudmessage");

        debugPointCloudPtr->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        debugPointCloudPtr->is_dense = true;
        debugPointCloudPtr->height = 1;
        debugPointCloudPtr->width = numInliers;
        // debugPointCloudPtr->header.seq = 0; 
        debugPointCloudPtr->header.stamp = rclcpp::Time();
        debugPointCloudPtr->header.frame_id = "zed_left_camera_frame";
        debugPointCloudPtr->data.resize(numInliers * sizeof(Point));
        auto pcPtr = reinterpret_cast<Point*>(debugPointCloudPtr->data.data());
        size_t i = 0;
        for (auto p: mFilteredPoints) {
            // calculate distance of each point from potential plane
            double distance = std::abs(mNormalInZEDVector.value().x() * p->x + mNormalInZEDVector.value().y() * p->y + mNormalInZEDVector.value().z() * p->z + mBestOffset);
            // double distance = std::abs(normal.x() * p->x + normal.y() * p->y + normal.z() * p->z + offset); //

            if (distance < distanceThreshold) {
                pcPtr[i].x = p->x;
                pcPtr[i].y = p->y;
                pcPtr[i].z = p->z;
                pcPtr[i].b = p->b;
                pcPtr[i].g = p->g;
                pcPtr[i].r = p->r;
                pcPtr[i].a = p->a;
                ++i;
            }
        }
        mDebugPCPub->publish(*debugPointCloudPtr);
    }

    void LanderAlign::ransac(double const distanceThreshold, int minInliers, int const epochs) {
        double offset;

        // define randomizer
        std::default_random_engine generator;
        std::uniform_int_distribution<int> distribution(0, (int) mFilteredPoints.size() - 1);

        if (mFilteredPoints.size() < 3) {
            mNormalInZEDVector = std::nullopt;
            mPlaneLocationInZEDVector = std::nullopt;
            return;
        }

        RCLCPP_INFO_STREAM(get_logger(), "before while");
        mNormalInZEDVector = std::make_optional<Eigen::Vector3d>(0, 0, 0);
        mPlaneLocationInZEDVector = std::make_optional<Eigen::Vector3d>(0, 0, 0);

        int numInliers = 0;
        while (mNormalInZEDVector.value().isZero()) { // TODO add give up condition after X iter
            // RCLCPP_INFO_STREAM(get_logger(), "Still 0");
            for (int i = 0; i < epochs; ++i) {
                // sample 3 random points (potential inliers)
                Point const* point1 = mFilteredPoints[distribution(generator)];
                Point const* point2 = mFilteredPoints[distribution(generator)];
                Point const* point3 = mFilteredPoints[distribution(generator)];

                Eigen::Vector3d vec1{point1->x, point1->y, point1->z};
                Eigen::Vector3d vec2{point2->x, point2->y, point2->z};
                Eigen::Vector3d vec3{point3->x, point3->y, point3->z};

                // fit a plane to these points
                Eigen::Vector3d normal = (vec1 - vec2).cross(vec1 - vec3).normalized();
                offset = -normal.dot(vec1); // calculate offset (D value) using one of the points
                RCLCPP_INFO_STREAM(get_logger(),"offset: "<< offset);
                RCLCPP_INFO_STREAM(get_logger(),"normal: "<< normal);

                numInliers = 0;

                for (auto p: mFilteredPoints) {
                    // calculate distance of each point from potential plane
                    double distance = std::abs(normal.x() * p->x + normal.y() * p->y + normal.z() * p->z + offset); //
                    
                    if (distance < distanceThreshold) {
                        ++numInliers; // count num of inliers that pass the "good enough fit" threshold
                    }
                }
                RCLCPP_INFO_STREAM(get_logger(), "numInliers: " << numInliers);
                RCLCPP_INFO_STREAM(get_logger(), "normal x " << normal.x());
                RCLCPP_INFO_STREAM(get_logger(), "minLiers " << minInliers);
                // update best plane if better inlier count
                if ((numInliers > minInliers) && (normal.x() != 0)) { 
                    RCLCPP_INFO_STREAM(get_logger(), "Updating plane");
                    minInliers = numInliers;
                    mNormalInZEDVector.value() = normal;
                    mBestOffset = offset;
                }
                RCLCPP_INFO_STREAM(get_logger(), "Condition Passed");
            }
        }

        // Run through one more loop to identify the center of the plane (one possibility for determining best center)
        numInliers = 0;
        mPlaneLocationInZEDVector = std::make_optional<Eigen::Vector3d>(Eigen::Vector3d::Zero());
        for (auto p: mFilteredPoints) {
            // calculate distance of each point from potential plane
            double distance = std::abs(mNormalInZEDVector.value().x() * p->x + mNormalInZEDVector.value().y() * p->y + mNormalInZEDVector.value().z() * p->z + mBestOffset);
            if (distance < distanceThreshold) {
                mPlaneLocationInZEDVector.value().x() += p->x;
                mPlaneLocationInZEDVector.value().y() += p->y;
                mPlaneLocationInZEDVector.value().z() += p->z;
                ++numInliers; // count num of inliers that pass the "good enough fit" threshold
            }
        }

        RCLCPP_INFO_STREAM(get_logger(), "after center calculation");

        if (numInliers == 0) {
            mNormalInZEDVector = std::nullopt;
            mPlaneLocationInZEDVector = std::nullopt;
            return;
        }

        //Average pnts
        mPlaneLocationInZEDVector.value() /= static_cast<float>(numInliers);

        uploadPC(numInliers, distanceThreshold);

        if (mNormalInZEDVector.value().x() > 0) mNormalInZEDVector.value() *= -1;

        mOffsetLocationInZEDVector = std::make_optional<Eigen::Vector3d>(mPlaneLocationInZEDVector.value() + mPlaneOffsetScalar * mNormalInZEDVector.value());

        SE3d zedToMap = SE3Conversions::fromTfTree(*mTfBuffer, mCameraFrameId, mMapFrameId);

        mNormalInWorldVector = std::make_optional<Eigen::Vector3d>(zedToMap.rotation() * mNormalInZEDVector.value());

        //Calculate the SO3 in the world frame
        Eigen::Matrix3d rot;
        Eigen::Vector3d forward = mNormalInZEDVector.value().normalized();
        Eigen::Vector3d worldUp = Eigen::Vector3d::UnitZ();
        Eigen::Vector3d left = worldUp.cross(forward);
        Eigen::Vector3d up = forward.cross(left);

        rot.col(0) = forward;
        rot.col(1) = left;
        rot.col(2) = up;

        RCLCPP_INFO_STREAM(get_logger(), "pre-publish");
        //Calculate the plane location in the world frame
        SE3d mPlaneLocationInZEDSE3d = {{mPlaneLocationInZEDVector.value().x(), mPlaneLocationInZEDVector.value().y(), mPlaneLocationInZEDVector.value().z()}, SO3d{Eigen::Quaterniond{rot}.normalized()}};
        mPlaneLocationInWorldSE3d = zedToMap * mPlaneLocationInZEDSE3d;
        mPlaneLocationInWorldVector = std::make_optional<Eigen::Vector3d>(mPlaneLocationInWorldSE3d.translation());

        //Calculate the offset location in the world frame
        SE3d mOffsetLocationInZEDSE3d = {{mOffsetLocationInZEDVector.value().x(), mOffsetLocationInZEDVector.value().y(), mOffsetLocationInZEDVector.value().z()}, SO3d{Eigen::Quaterniond{rot}.normalized()}};
        mOffsetLocationInWorldSE3d = zedToMap * mOffsetLocationInZEDSE3d;
        mOffsetLocationInWorldVector = std::make_optional<Eigen::Vector3d>(mOffsetLocationInWorldSE3d.translation());

        //Push to the tf tree
        SE3Conversions::pushToTfTree(*mTfBroadcaster, "plane", mMapFrameId, mPlaneLocationInWorldSE3d, get_clock()->now());

        SE3Conversions::pushToTfTree(*mTfBroadcaster, "offset", mMapFrameId, mOffsetLocationInWorldSE3d, get_clock()->now());

        RCLCPP_INFO_STREAM(get_logger(), "post publish");

        //Compare Rover Location to Target Location
        if(mOffsetLocationInZEDSE3d.translation().x() < 0) mNormalInZEDVector = std::nullopt;
    }

    /* void LanderAlign::sendTwist() { // We currently don't call this and it has errors
        //Locations
        Eigen::Vector3d rover_dir;

        //Final msg
        geometry_msgs::Twist twist;

        //Thresholds
        float const linear_thresh = 0.01; // could be member variables
        float const angular_thresh = 0.001;


        ros::Rate rate(20); // ROS Rate at 20Hzn::Matrix3d roverToPlaneNorm;
        
        while (ros::ok()) {
            // If the client has cancelled the server stop moving
            // if(mActionServer->isPreemptRequested()){
            //     mActionServer->setPreempted();
            //     twist.angular.z = 0;
            //     twist.linear.x = 0;
            //     mTwistPub.publish(twist);
            //     break;
            // }
                
            // Publish the current state of the RTR controller 
        	LanderAlignFeedback feedback;
			feedback.curr_state = RTRSTRINGS[static_cast<std::size_t>(mLoopState)];
			mActionServer->publishFeedback(feedback);
            
            // Push plane and offset locations for debugging
            SE3Conversions::pushToTfTree(mTfBroadcaster, "plane", mMapFrameId, mPlaneLocationInWorldSE3d);
            SE3Conversions::pushToTfTree(mTfBroadcaster, "offset", mMapFrameId, mOffsetLocationInWorldSE3d);
            
            SE3d roverInWorld = SE3Conversions::fromTfTree(*mTfBuffer, "base_link", "map");
            Eigen::Vector3d roverPosInWorld{(roverInWorld.translation().x()), (roverInWorld.translation().y()), 0.0};
            Eigen::Vector3d targetPosInWorld;

            if (mLoopState == RTRSTATE::turn1) {
                targetPosInWorld = mOffsetLocationInWorldVector.value();
            } else if (mLoopState == RTRSTATE::turn2) {
                targetPosInWorld = mPlaneLocationInWorldVector.value();
            }
            targetPosInWorld.z() = 0;

            Eigen::Vector3d roverToTargetForward = targetPosInWorld - roverPosInWorld;
            roverToTargetForward.normalize();

            Eigen::Vector3d up = Eigen::Vector3d::UnitZ();
            Eigen::Vector3d left = up.cross(roverToTargetForward);

            Eigen::Matrix3d roverToTargetMat;
            roverToTargetMat.col(0) = roverToTargetForward;
            roverToTargetMat.col(1) = up;
            roverToTargetMat.col(2) = left;

            //SO3 Matrices for lie algebra
            SO3d roverToTargetSO3 = SE3Conversions::fromColumns(roverToTargetForward, left, up);
            SO3d roverSO3 = roverInWorld.asSO3();

            Eigen::Vector3d distanceToTargetVector = targetPosInWorld - Eigen::Vector3d{roverInWorld.translation().x(), roverInWorld.translation().y(), roverInWorld.translation().z()};

            double distanceToTarget = std::abs(distanceToTargetVector.norm());

            manif::SO3Tangentd SO3tan = roverToTargetSO3 - roverSO3; // 3 x 1 matrix of angular velocities (x,y,z)

            switch (mLoopState) {
                case RTRSTATE::turn1: {
                    if (distanceToTarget < 0.5) mLoopState = RTRSTATE::turn2;
                    

                    double angle_rate = mAngleP * SO3tan.z();
                    angle_rate = (std::abs(angle_rate) > mAngleFloor) ? angle_rate : copysign(mAngleFloor, angle_rate);

                    ROS_INFO("w_z velocity %f", SO3tan.z());

                    twist.angular.z = angle_rate;

                    if (std::abs(SO3tan.z()) < angular_thresh) {
                        mLoopState = RTRSTATE::drive;
                        twist.angular.z = 0;
                        twist.linear.x = 0;
                        ROS_INFO("Done spinning");
                    }
                    // ROS_INFO("In state: turning to point...");
                    break;
                }

                case RTRSTATE::drive: {
                    if (std::abs(SO3tan.z()) > angular_thresh) {
                        mLoopState = RTRSTATE::turn1;
                        ROS_INFO("Rotation got off");
                        twist.linear.x = 0;
                    }
                    double driveRate = std::min(mLinearP * distanceToTarget, 1.0);

                    ROS_INFO("distance: %f", distanceToTarget);

                    twist.linear.x = driveRate;

                    ROS_INFO("distance to target %f", distanceToTarget);

                    if (std::abs(distanceToTarget) < linear_thresh) {
                        mLoopState = RTRSTATE::turn2;
                        twist.linear.x = 0;
                        twist.angular.z = 0;
                    }
                    // ROS_INFO("In state: driving to point...");
                    break;
                }

                case RTRSTATE::turn2: {
                    double angle_rate = mAngleP * SO3tan.z();
                    angle_rate = (std::abs(angle_rate) > mAngleFloor) ? angle_rate : copysign(mAngleFloor, angle_rate);
                    twist.angular.z = angle_rate;
                    twist.linear.x = 0;


                    if (std::abs(SO3tan.z()) < angular_thresh) {
                        mLoopState = RTRSTATE::done;
                        twist.angular.z = 0;
                        twist.linear.x = 0;
                        //  ROS_INFO("Done turning to lander");
                    }
                    // ROS_INFO("In state: turning to lander...");
                    break;
                }

                case RTRSTATE::done: {
                    twist.linear.x = 0;
                    twist.angular.z = 0;
                    break;
					mCloud = nullptr;
                }
            }
            // ROS_INFO("THE TWIST IS: Angular: %f, with linear %f,", twist.angular.z, twist.linear.x);
            RCLCPP_INFO_STREAM(get_logger(), mLoopState);
            mTwistPub.publish(twist);
            if (mLoopState == RTRSTATE::done) {
                break;
            }
        }
     }*/
    

    auto LanderAlign::createSpline(int density, double offset) -> bool{
        //Constants
        const double kSplineStart = 7.0/8;
        const double dOmega = 0;
        const double dVelocity = 1;

        // Calculate the angle to the world
        const double dAngle = calcAngleWithWorldX(-mNormalInWorldVector.value());

        //Calculate the spline length
        SE3d planeInRover = SE3Conversions::fromTfTree(*mTfBuffer, "plane", mCameraFrameId);
        double xDistanceFromRoverToPlane = planeInRover.translation().x();
        double splineLength = kSplineStart * xDistanceFromRoverToPlane;

        if(xDistanceFromRoverToPlane <= offset/kSplineStart){
            return false;
        }
        
        // Append all of the points to each other
        // Eigen::Vector3d baseSplinePoint = mPlaneLocationInWorldVector.value() + splineLength * mNormalInWorldVector.value();
        Eigen::Vector3d baseSplinePoint = mPlaneLocationInWorldVector.value() + splineLength * mNormalInWorldVector.value();
        Eigen::Vector3d densityVector = mNormalInWorldVector.value() / density;
        Eigen::Vector3d splinePoint = Eigen::Vector3d::Zero();
        
        while(splinePoint.norm() < (splineLength - offset)){
            // Eigen::Vector3d splinePointInWorld = baseSplinePoint - splinePoint;
            Eigen::Vector3d splinePointInWorld = baseSplinePoint - splinePoint;
            // Create the new point to be added to the vector
            Vector5d newPoint;
            newPoint << splinePointInWorld.x(),
                        splinePointInWorld.y(),
                        dAngle,
                        dVelocity,
                        dOmega;
                        
            mPathPoints.emplace_back(newPoint);
            splinePoint = splinePoint + densityVector;
        }

        return true;
    }

    void LanderAlign::publishSpline(){
        Eigen::Matrix3d rot;
        rot <<  1, 0, 0,
                0, 1, 0,
                0, 0, 1;
        int index = 0;
        for(auto const & point : mPathPoints){
            SE3d mPlaneLocationInZEDSE3d = {{point.coeff(0,0), point.coeff(1,0), 0}, SO3d{Eigen::Quaterniond{rot}.normalized()}};
            SE3Conversions::pushToTfTree(*mTfBroadcaster, std::format("point_{}", index), mMapFrameId, mPlaneLocationInZEDSE3d, get_clock()->now());
            index++;
        }
    }

    // ACTION SERVER FUNCTIONS
    rclcpp_action::GoalResponse LanderAlign::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const action::LanderAlign::Goal> goal) {
        RCLCPP_INFO_STREAM(get_logger(), "in handle goal");

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } 

    rclcpp_action::CancelResponse LanderAlign::handle_cancel(std::shared_ptr<GoalHandleLanderAlign> goal_handle){
        RCLCPP_INFO_STREAM(get_logger(), "in handle cancel");

        (void)goal_handle;
        driveTwist.angular.z = 0;
        driveTwist.linear.x = 0;
        mTwistPub->publish(driveTwist);
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void LanderAlign::handle_accepted(std::shared_ptr<GoalHandleLanderAlign> goal_handle) {
        RCLCPP_INFO_STREAM(get_logger(), "in handle accepted");

        std::thread{&LanderAlign::execute, this, goal_handle}.detach();
    }

    auto LanderAlign::execute(const std::shared_ptr<GoalHandleLanderAlign> goal_handle) -> void {
        RCLCPP_INFO_STREAM(get_logger(), "in execute");
        ActionServerCallBack(goal_handle);

    }

    LanderAlign::~LanderAlign() {}

} // namespace mrover


auto main(int argc, char* argv[]) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::LanderAlign>());
    rclcpp::shutdown();
    return 0;
}

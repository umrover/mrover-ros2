#pragma once

// C++ Standard Library Headers, std namespace


// ROS Headers, ros namespace
#include "pch.hpp"


namespace mrover {

    /**
     *  Starter project perception node
     *
     *  Input:  Image data, just RGB pixels.
     *  Output: ArUco tag pixel coordinates that is closest to the center of the camera.
     *          Also an approximation for how far away the tag is.
     */
    class IEKF : public rclcpp::Node {
    private:

    public:
        IEFK();

        void CorrectRightInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H,
                                   const Eigen::MatrixXd& N); // ErrorType error_type

        void CorrectLeftInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H,
                          const Eigen::MatrixXd& N); // ErrorType error_type

        
    };

} // namespace mrover
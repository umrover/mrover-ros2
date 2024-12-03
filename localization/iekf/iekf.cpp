#include "iekf.hpp"

namespace mrover {
    
    IEKF::IEKF() : Node{"iekf"} {
        // TODO: pubs and subs and buffers(?)
    }

    auto IEKF::CorrectRightInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H,
                                const Eigen::MatrixXd& N) -> void {
        
    }

    auto IEKF::CorrectLeftInvariant(const Eigen::MatrixXd& Z, const Eigen::MatrixXd& H,
                                const Eigen::MatrixXd& N) -> void {
        
    }

    auto IEKF::gyroCallback(geometry_msgs::msg::Vector3Stamped vel) -> void {
        manif::SO3f heading = X.block<3,3>(0,0);
        auto heading_updated = heading.plus(vel);
    }

    auto IEKF::accelCallback(geometry_msgs::msg::Vector3Stamped accel) -> void {
        
    }

    auto magCallback() -> void {

    }

    auto gpsCallback() -> void {

    }

}
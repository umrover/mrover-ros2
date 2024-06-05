#pragma once

#define GLFW_INCLUDE_NONE

#include <bit>
#include <charconv>
#include <chrono>
#include <execution>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <source_location>
#include <span>
#include <stdexcept>
#include <thread>
#include <unordered_set>

#include <boost/circular_buffer.hpp>
#include <boost/container/static_vector.hpp>
#include <boost/range/combine.hpp>
#include <boost/thread.hpp>
#include <boost/thread/future.hpp>
#include <boost/bimap.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
//#include <nodelet/loader.h>
//#include <nodelet/nodelet.h>
//#include <ros/package.h>
//#include <ros/serialization.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>
//#include <xmlrpcpp/XmlRpcValue.h>

#include <glfw3webgpu.h>

#include <webgpu.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Geometry>

#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_wgpu.h>

#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyGearConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointMotor.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletDynamics/Featherstone/btMultiBodyMLCPConstraintSolver.h>
#include <BulletDynamics/MLCPSolvers/btDantzigSolver.h>
#include <BulletDynamics/MLCPSolvers/btMLCPSolver.h>
#include <btBulletDynamicsCommon.h>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <lie.hpp>
#include <loop_profiler.hpp>
#include <manif/algorithms/interpolation.h>
#include <manif/manif.h>
#include <params_utils.hpp>
#include <point.hpp>
#include <units/units.hpp>

#include <mrover/CalibrationStatus.h>
#include <mrover/ControllerState.h>
#include <mrover/IK.h>
#include <mrover/ImageTarget.h>
#include <mrover/ImageTargets.h>
#include <mrover/Position.h>
#include <mrover/Throttle.h>
#include <mrover/Velocity.h>

#include "tag_detector.hpp"

namespace mrover {

    TagDetectorNodeletBase::TagDetectorNodeletBase(std::string const& name) : Node{name, rclcpp::NodeOptions{}.use_intra_process_comms(true)} {

        declare_parameter("world_frame", rclcpp::ParameterType::PARAMETER_STRING);
        declare_parameter("camera_frame", rclcpp::ParameterType::PARAMETER_STRING);
        // False positive parameters
        declare_parameter("min_hit_count_before_publish", rclcpp::ParameterType::PARAMETER_INTEGER);
        declare_parameter("max_hit_count", rclcpp::ParameterType::PARAMETER_INTEGER);
        declare_parameter("increment_weight", rclcpp::ParameterType::PARAMETER_INTEGER);
        declare_parameter("decrement_weight", rclcpp::ParameterType::PARAMETER_INTEGER);
        // Detection parameters
        declare_parameter("dictionary", rclcpp::ParameterType::PARAMETER_INTEGER);
        declare_parameter("adaptive_thresh_constant", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("adaptive_thresh_win_size_max", rclcpp::ParameterType::PARAMETER_INTEGER);
        declare_parameter("adaptive_thresh_win_size_min", rclcpp::ParameterType::PARAMETER_INTEGER);
        declare_parameter("adaptive_thresh_win_size_step", rclcpp::ParameterType::PARAMETER_INTEGER);
        declare_parameter("corner_refinement_max_iterations", rclcpp::ParameterType::PARAMETER_INTEGER);
        declare_parameter("corner_refinement_min_accuracy", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("corner_refinement_win_size", rclcpp::ParameterType::PARAMETER_INTEGER);
        declare_parameter("do_corner_refinement", rclcpp::ParameterType::PARAMETER_BOOL);
        declare_parameter("corner_refinement_subpix", rclcpp::ParameterType::PARAMETER_BOOL);
        declare_parameter("error_correction_rate", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("min_corner_distance_rate", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("marker_border_bits", rclcpp::ParameterType::PARAMETER_INTEGER);
        declare_parameter("max_erroneous_bits_in_border_rate", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("min_distance_to_border", rclcpp::ParameterType::PARAMETER_INTEGER);
        declare_parameter("min_marker_distance_rate", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("min_marker_perimeter_rate", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("max_marker_perimeter_rate", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("min_otsu_std_dev", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("perspective_remove_ignored_margin_per_cell", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("perspective_remove_pixel_per_cell", rclcpp::ParameterType::PARAMETER_INTEGER);
        declare_parameter("polygonal_approx_accuracy_rate", rclcpp::ParameterType::PARAMETER_DOUBLE);

        auto dictionaryNumber = static_cast<int>(get_parameter("dictionary").as_int());
        mDictionary = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(dictionaryNumber));

        mDetectorParams = cv::makePtr<cv::aruco::DetectorParameters>();

        mDetectedImagePub = create_publisher<sensor_msgs::msg::Image>("tag_detection", 1);

        auto paramSub = std::make_shared<rclcpp::ParameterEventHandler>(this);
        paramSub->add_parameter_callback("adaptive_thresh_constant", [this](rclcpp::Parameter const& param) {
            mDetectorParams->adaptiveThreshConstant = param.as_double();
        });
        paramSub->add_parameter_callback("adaptive_thresh_win_size_max", [this](rclcpp::Parameter const& param) {
            mDetectorParams->adaptiveThreshWinSizeMax = static_cast<int>(param.as_int());
        });
        paramSub->add_parameter_callback("adaptive_thresh_win_size_min", [this](rclcpp::Parameter const& param) {
            mDetectorParams->adaptiveThreshWinSizeMin = static_cast<int>(param.as_int());
        });
        paramSub->add_parameter_callback("adaptive_thresh_win_size_step", [this](rclcpp::Parameter const& param) {
            mDetectorParams->adaptiveThreshWinSizeStep = static_cast<int>(param.as_int());
        });
        paramSub->add_parameter_callback("corner_refinement_max_iterations", [this](rclcpp::Parameter const& param) {
            mDetectorParams->cornerRefinementMaxIterations = static_cast<int>(param.as_int());
        });
        paramSub->add_parameter_callback("corner_refinement_min_accuracy", [this](rclcpp::Parameter const& param) {
            mDetectorParams->cornerRefinementMinAccuracy = param.as_double();
        });
        paramSub->add_parameter_callback("corner_refinement_win_size", [this](rclcpp::Parameter const& param) {
            mDetectorParams->cornerRefinementWinSize = static_cast<int>(param.as_int());
        });
        paramSub->add_parameter_callback("do_corner_refinement", [this](rclcpp::Parameter const& param) {
            mDetectorParams->cornerRefinementMethod = param.as_bool() ? cv::aruco::CORNER_REFINE_SUBPIX : cv::aruco::CORNER_REFINE_NONE;
        });
        paramSub->add_parameter_callback("error_correction_rate", [this](rclcpp::Parameter const& param) {
            mDetectorParams->errorCorrectionRate = param.as_double();
        });
        paramSub->add_parameter_callback("min_corner_distance_rate", [this](rclcpp::Parameter const& param) {
            mDetectorParams->minCornerDistanceRate = param.as_double();
        });
        paramSub->add_parameter_callback("marker_border_bits", [this](rclcpp::Parameter const& param) {
            mDetectorParams->markerBorderBits = static_cast<int>(param.as_int());
        });
        paramSub->add_parameter_callback("max_erroneous_bits_in_border_rate", [this](rclcpp::Parameter const& param) {
            mDetectorParams->maxErroneousBitsInBorderRate = param.as_double();
        });
        paramSub->add_parameter_callback("min_distance_to_border", [this](rclcpp::Parameter const& param) {
            mDetectorParams->minDistanceToBorder = static_cast<int>(param.as_int());
        });
        paramSub->add_parameter_callback("min_marker_distance_rate", [this](rclcpp::Parameter const& param) {
            mDetectorParams->minMarkerDistanceRate = param.as_double();
        });
        paramSub->add_parameter_callback("min_marker_perimeter_rate", [this](rclcpp::Parameter const& param) {
            mDetectorParams->minMarkerPerimeterRate = param.as_double();
        });
        paramSub->add_parameter_callback("max_marker_perimeter_rate", [this](rclcpp::Parameter const& param) {
            mDetectorParams->maxMarkerPerimeterRate = param.as_double();
        });
        paramSub->add_parameter_callback("min_otsu_std_dev", [this](rclcpp::Parameter const& param) {
            mDetectorParams->minOtsuStdDev = param.as_double();
        });
        paramSub->add_parameter_callback("perspective_remove_ignored_margin_per_cell", [this](rclcpp::Parameter const& param) {
            mDetectorParams->perspectiveRemoveIgnoredMarginPerCell = param.as_double();
        });
        paramSub->add_parameter_callback("perspective_remove_pixel_per_cell", [this](rclcpp::Parameter const& param) {
            mDetectorParams->perspectiveRemovePixelPerCell = static_cast<int>(param.as_int());
        });
        paramSub->add_parameter_callback("polygonal_approx_accuracy_rate", [this](rclcpp::Parameter const& param) {
            mDetectorParams->polygonalApproxAccuracyRate = param.as_double();
        });

        RCLCPP_INFO(get_logger(), "Tag detection ready!");
    }

    StereoTagDetectorNodelet::StereoTagDetectorNodelet() : TagDetectorNodeletBase{"stereo_tag_detector"} {
        mPointCloudSub = create_subscription<sensor_msgs::msg::PointCloud2>("points", 1, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) {
            pointCloudCallback(msg);
        });
    }

    ImageTagDetectorNodelet::ImageTagDetectorNodelet() : TagDetectorNodeletBase{"image_tag_detector"} {
        declare_parameter("camera_horizontal_fov", rclcpp::ParameterType::PARAMETER_DOUBLE);

        mTargetsPub = create_publisher<msg::ImageTargets>("tags", 1);

        mImageSub = create_subscription<sensor_msgs::msg::Image>("image", 1, [this](sensor_msgs::msg::Image::ConstSharedPtr const& msg) {
            imageCallback(msg);
        });
    }

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::StereoTagDetectorNodelet>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}

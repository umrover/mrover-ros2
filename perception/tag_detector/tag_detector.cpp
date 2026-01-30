#include "tag_detector.hpp"

namespace mrover {

    TagDetectorBase::TagDetectorBase(std::string const& name, rclcpp::NodeOptions const& options) : Node{name, options}, mMinTagHitCountBeforePublish{}, mMaxTagHitCount{}, mTagIncrementWeight{}, mTagDecrementWeight{} {

        int dictionaryNumber{};
        bool doCornerRefinement{};
        bool cornerRefinementSubPix{};

        mDetectorParams = cv::makePtr<cv::aruco::DetectorParameters>();

        std::vector<ParameterWrapper> params{
                {"world_frame", mMapFrameId, "map"},
                {"camera_frame", mCameraFrameId, "zed_left_camera_frame"},
                {"min_hit_count_before_publish", mMinTagHitCountBeforePublish, 5},
                {"max_hit_count", mMaxTagHitCount, 5},
                {"increment_weight", mTagIncrementWeight, 2},
                {"decrement_weight", mTagDecrementWeight, 1},
                {"dictionary", dictionaryNumber, 0},
                {"adaptive_thresh_constant", mDetectorParams->adaptiveThreshConstant, 7.0},
                {"adaptive_thresh_win_size_max", mDetectorParams->adaptiveThreshWinSizeMax, 23},
                {"adaptive_thresh_win_size_min", mDetectorParams->adaptiveThreshWinSizeMin, 3},
                {"adaptive_thresh_win_size_step", mDetectorParams->adaptiveThreshWinSizeStep, 10},
                {"corner_refinement_max_iterations", mDetectorParams->cornerRefinementMaxIterations, 30},
                {"corner_refinement_min_accuracy", mDetectorParams->cornerRefinementMinAccuracy, 0.1},
                {"corner_refinement_win_size", mDetectorParams->cornerRefinementWinSize, 5},
                {"do_corner_refinement", doCornerRefinement, true},
                {"corner_refinement_subpix", cornerRefinementSubPix, true},
                {"error_correction_rate", mDetectorParams->errorCorrectionRate, 0.6},
                {"min_corner_distance_rate", mDetectorParams->minCornerDistanceRate, 0.05},
                {"marker_border_bits", mDetectorParams->markerBorderBits, 1},
                {"max_erroneous_bits_in_border_rate", mDetectorParams->maxErroneousBitsInBorderRate, 0.35},
                {"min_distance_to_border", mDetectorParams->minDistanceToBorder, 3},
                {"min_marker_distance_rate", mDetectorParams->minMarkerDistanceRate, 0.05},
                {"min_marker_perimeter_rate", mDetectorParams->minMarkerPerimeterRate, 0.03},
                {"max_marker_perimeter_rate", mDetectorParams->maxMarkerPerimeterRate, 4.0},
                {"min_otsu_std_dev", mDetectorParams->minOtsuStdDev, 5.0},
                {"perspective_remove_ignored_margin_per_cell", mDetectorParams->perspectiveRemoveIgnoredMarginPerCell, 0.13},
                {"perspective_remove_pixel_per_cell", mDetectorParams->perspectiveRemovePixelPerCell, 4},
                {"polygonal_approx_accuracy_rate", mDetectorParams->polygonalApproxAccuracyRate, 0.08}};

        ParameterWrapper::declareParameters(this, params);

        mDictionary = cv::makePtr<cv::aruco::Dictionary>(cv::aruco::getPredefinedDictionary(dictionaryNumber));

        if (doCornerRefinement) {
            if (cornerRefinementSubPix) {
                mDetectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
            } else {
                mDetectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
            }
        } else {
            mDetectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
        }


        RCLCPP_INFO(get_logger(), "Tag detection ready!");
    }

    StereoTagDetector::StereoTagDetector(rclcpp::NodeOptions const& options) : TagDetectorBase{"stereo_tag_detector", options} {
        mDetectedImagePub = create_publisher<sensor_msgs::msg::Image>("stereo_tag_detection", 1);

        mPointCloudSub = create_subscription<sensor_msgs::msg::PointCloud2>("/zed/left/points", 1, [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr const& msg) {
            pointCloudCallback(msg);
        });
    }

    ImageTagDetector::ImageTagDetector(rclcpp::NodeOptions const& options) : TagDetectorBase{"image_tag_detector", options}, mCameraHorizontalFOV{} {
        std::string imageSubTopic;
        std::vector<ParameterWrapper> params{
                {"image_sub_topic", imageSubTopic, "/long_range_cam/image"},
                {"camera_horizontal_fov", mCameraHorizontalFOV, 10.0}};

        ParameterWrapper::declareParameters(this, params);

        mDetectedImagePub = create_publisher<sensor_msgs::msg::Image>(std::format("/{}/detections", get_name()), 1);

        mTargetsPub = create_publisher<msg::ImageTargets>(std::format("/{}/tags", get_name()), 1);

        mImageSub = create_subscription<sensor_msgs::msg::Image>(imageSubTopic, 1, [this](sensor_msgs::msg::Image::ConstSharedPtr const& msg) {
            imageCallback(msg);
        });
    }
} // namespace mrover

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::StereoTagDetector)
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::ImageTagDetector)

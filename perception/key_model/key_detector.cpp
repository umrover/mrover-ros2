#include "key_detector.hpp"
#include <functional>

namespace mrover {

    KeyDetectorBase::KeyDetectorBase(rclcpp::NodeOptions const& options) : rclcpp::Node(NODE_NAME, options), mLoopProfiler{get_logger()} {
        std::string keyDetectionModelName;
        std::string textCoordsModelName;

        std::vector<ParameterWrapper> params{
                {"camera_frame", mCameraFrame, "zed_left_camera_frame"},
                {"world_frame", mWorldFrame, "map"},
                {"increment_weight", mObjIncrementWeight, 2},
                {"decrement_weight", mObjDecrementWeight, 1},
                {"hitcount_threshold", mObjHitThreshold, 5},
                {"hitcount_max", mObjMaxHitcount, 10},
                {"key_detection_model_name", keyDetectionModelName, "yolo-key-detection"},
                {"text_coords_model_name", textCoordsModelName, "magic"},
                {"yolo_model_score_threshold", mModelScoreThreshold, 0.3},
                {"yolo_model_nms_threshold", mModelNMSThreshold, 0.3},
                {"object_detector_debug", mDebug, true}};

        ParameterWrapper::declareParameters(this, params);

        // All of these variables will be invalidated after calling this function

        std::filesystem::path packagePath = std::filesystem::path{ament_index_cpp::get_package_prefix("mrover")} / ".." / ".." / "src" / "mrover";

        // Initialize TensorRT Inference Object and Get Important Output Information
        mKeyDetectionTensorRT = TensortRT{keyDetectionModelName, packagePath.string()};
        mTextCoordsTensorRT = TensortRT{textCoordsModelName, packagePath.string()};

        using namespace std::placeholders;

        mKeyDetectionModel = Model(this, keyDetectionModelName, {0}, {"key"}, mKeyDetectionTensorRT.getInputTensorSize(), mKeyDetectionTensorRT.getOutputTensorSize(), [](Model const& model, cv::Mat const& input, cv::Mat& output) { preprocessYOLOv8Input(model, input, output); }, [](Model const& model, cv::Mat& input) { postprocessYOLOv8Output(model, input); });

        mTextCoordModel = Model(this, textCoordsModelName, {}, {}, mTextCoordsTensorRT.getInputTensorSize(), mTextCoordsTensorRT.getOutputTensorSize(), [](Model const& model, cv::Mat const& input, cv::Mat& output) { preprocessTextCoordsInput(model, input, output); }, [](Model const& model, cv::Mat& input) { postprocessTextCoordsOutput(model, input); });

        RCLCPP_INFO_STREAM(get_logger(), std::format("Object detector initialized with model: {} and thresholds: {} and {}", mKeyDetectionModel.modelName, mModelScoreThreshold, mModelNMSThreshold));

        mTextCoordsBlob = cv::Mat{static_cast<int>(mTextCoordsTensorRT.getInputTensorSize()[2]), static_cast<int>(mTextCoordsTensorRT.getInputTensorSize()[3]), CV_32FC3};
    }

    ImageKeyDetector::ImageKeyDetector(rclcpp::NodeOptions const& options) : KeyDetectorBase(options) {
        RCLCPP_INFO_STREAM(get_logger(), "Creating Image Object Detector...");

        std::vector<ParameterWrapper> params{
                {"long_range_camera/fov", mCameraHorizontalFov, 80.0}};

        ParameterWrapper::declareParameters(this, params);

        mDebugImgPub = create_publisher<sensor_msgs::msg::Image>("/long_range_object_detector/debug_img", 1);

        mSensorSub = create_subscription<sensor_msgs::msg::Image>("/long_range_cam/image", 1, [this](sensor_msgs::msg::Image::ConstSharedPtr const& msg) {
            ImageKeyDetector::imageCallback(msg);
        });

        mTargetsPub = create_publisher<mrover::msg::ImageTargets>("objects", 1);

        // Create our palette (reminder, convert from pair to CV stuff)
        palette[std::make_pair(43.02179054054054, 27.980405405405406)] = "lalt";
        palette[std::make_pair(91.65040106951872, 32.66466131907308)] = "space";
        palette[std::make_pair(139.88621715418938, 27.172285572632624)] = "ralt";
        palette[std::make_pair(157.83941222776176, 27.047494096037784)] = "fn";
        palette[std::make_pair(175.63085320310256, 27.414536052858374)] = "menu";
        palette[std::make_pair(193.72220416124838, 27.325747724317296)] = "rctrl";
        palette[std::make_pair(215.53828920570265, 27.030753564154786)] = "left";
        palette[std::make_pair(48.28085924975954, 66.35139467778134)] = "X";
        palette[std::make_pair(62.17908059354088, 66.41838812918243)] = "C";
        palette[std::make_pair(228.94114809454896, 27.195610226724554)] = "down";
        palette[std::make_pair(75.91751746372918, 66.30548092423429)] = "V";
        palette[std::make_pair(241.88837478311163, 27.330248698669752)] = "right";
        palette[std::make_pair(89.71553250695675, 66.25309891221856)] = "B";
        palette[std::make_pair(103.58842285994604, 66.31089036055924)] = "N";
        palette[std::make_pair(117.28655888030887, 66.25977316602317)] = "M";
        palette[std::make_pair(130.94723496739917, 66.07787973919343)] = "<";
        palette[std::make_pair(144.7671333824613, 66.13559322033899)] = ">";
        palette[std::make_pair(158.13381369016983, 66.265697375193)] = "?";
        palette[std::make_pair(41.28372409106354, 107.85168195718654)] = "S";
        palette[std::make_pair(183.29111661166115, 67.51127612761276)] = "rshift";
        palette[std::make_pair(54.957324455205814, 107.81492130750605)] = "D";
        palette[std::make_pair(229.1394869445717, 66.22881355932203)] = "up";
        palette[std::make_pair(68.83254651485699, 107.75701194112746)] = "F";
        palette[std::make_pair(82.61242834809796, 107.72120896300156)] = "G";
        palette[std::make_pair(96.44725790513834, 107.69404644268775)] = "H";
        palette[std::make_pair(110.24620024125453, 107.7021712907117)] = "J";
        palette[std::make_pair(124.05666027791088, 107.67920459990417)] = "K";
        palette[std::make_pair(137.8968544257498, 107.67959034381857)] = "L";
        palette[std::make_pair(151.63998983739836, 107.69347052845528)] = ";";
        palette[std::make_pair(37.3521786100108, 148.61973352538712)] = "W";
        palette[std::make_pair(165.45633359559403, 107.77130868082874)] = "\"";
        palette[std::make_pair(51.22231012658228, 148.6368670886076)] = "E";
        palette[std::make_pair(65.08666087962963, 148.60749421296296)] = "R";
        palette[std::make_pair(186.96253193301163, 107.70394550099347)] = "enter";
        palette[std::make_pair(78.94241119483316, 148.56620021528525)] = "T";
        palette[std::make_pair(92.78344757552678, 148.52652957603453)] = "Y";
        palette[std::make_pair(106.58021521154316, 148.5008559550012)] = "U";
        palette[std::make_pair(120.41790138450328, 148.52028175856205)] = "I";
        palette[std::make_pair(134.20103346456693, 148.53272637795277)] = "O";
        palette[std::make_pair(46.30736989172197, 187.83670974502272)] = "3";
        palette[std::make_pair(147.99747347145023, 148.5456038403234)] = "P";
        palette[std::make_pair(161.72080340516095, 148.52833200319233)] = "[";
        palette[std::make_pair(60.18369259606373, 187.73758200562324)] = "4";
        palette[std::make_pair(175.5483962531933, 148.62148736871984)] = "]";
        palette[std::make_pair(73.96954756380511, 187.71200696055683)] = "5";
        palette[std::make_pair(192.71439604572134, 148.74003707136237)] = "|";
        palette[std::make_pair(87.64202069716775, 187.87608932461873)] = "6";
        palette[std::make_pair(228.3690640848317, 151.77201475334255)] = "end";
        palette[std::make_pair(214.97147915027537, 151.7002360346184)] = "del";
        palette[std::make_pair(241.1488384656942, 151.89276066990817)] = "pg dn";
        palette[std::make_pair(101.45172233820459, 187.93162839248436)] = "7";
        palette[std::make_pair(115.16089743589744, 187.99858974358975)] = "8";
        palette[std::make_pair(128.98369842078452, 188.08571064696892)] = "9";
        palette[std::make_pair(32.44383810823101, 231.1043656207367)] = "f1";
        palette[std::make_pair(142.71432305279666, 188.05697856769473)] = "0";
        palette[std::make_pair(45.91297591297592, 231.43589743589743)] = "f2";
        palette[std::make_pair(156.1441809757427, 187.69010629599344)] = "-";
        palette[std::make_pair(59.38028661307235, 231.26074799021322)] = "f3";
        palette[std::make_pair(169.9335461404527, 187.77539175856066)] = "+";
        palette[std::make_pair(73.00543825975687, 231.09580934101086)] = "f4";
        palette[std::make_pair(189.8590541381456, 187.69368388301183)] = "back";
        palette[std::make_pair(241.5061331775701, 188.70093457943926)] = "pg up";
        palette[std::make_pair(228.46753875968992, 188.9188468992248)] = "home";
        palette[std::make_pair(215.07946465913844, 188.94165621079046)] = "insert";
        palette[std::make_pair(94.22209268434858, 231.09938793354706)] = "f5";
        palette[std::make_pair(107.62722143864598, 231.05909732016926)] = "f6";
        palette[std::make_pair(120.9877334820184, 231.02522999721216)] = "f7";
        palette[std::make_pair(133.39643366619114, 231.11968616262482)] = "f8";
        palette[std::make_pair(155.3114311431143, 231.05880588058807)] = "f9";
        palette[std::make_pair(168.86315956770503, 231.09297520661158)] = "f10";
        palette[std::make_pair(182.2447504302926, 231.0185886402754)] = "f11";
        palette[std::make_pair(195.22088122605365, 230.98371647509578)] = "f12";
        palette[std::make_pair(241.37142857142857, 231.23105590062113)] = "pause";
        palette[std::make_pair(214.86027397260273, 231.50890410958905)] = "print";
        palette[std::make_pair(228.22222222222223, 231.3496577145866)] = "lock";
        palette[std::make_pair(7.19589552238806, 27.02771855010661)] = "lctrl";
        palette[std::make_pair(24.874324885749896, 27.245118404653095)] = "windows";
        palette[std::make_pair(13.61627408993576, 66.67922912205567)] = "lshift";
        palette[std::make_pair(34.39268651231165, 66.34105108416024)] = "Z";
        palette[std::make_pair(9.378082829222894, 107.4164727780363)] = "caps";
        palette[std::make_pair(28.95089984350548, 107.77582159624413)] = "A";
        palette[std::make_pair(23.54336842105263, 148.54842105263157)] = "Q";
        palette[std::make_pair(32.344192634560905, 187.77195467422095)] = "2";
        palette[std::make_pair(7.2325, 148.16325)] = "ltab";
        palette[std::make_pair(18.651041666666668, 187.49337121212122)] = "1";
        palette[std::make_pair(5.163086489542114, 187.15404183154325)] = "`";
        palette[std::make_pair(5.749533291848164, 230.97293092719352)] = "esc";

        cv_palette = cv::Mat_(87, 2, 1);
        int index = 0;
        for (auto &i: palette)
        {
            cv_palette.at<double>(index, 0) = (i.first.first);
            cv_palette.at<double>(index, 1) = (i.first.second);
            index++;
        }

        for (const auto& entry : palette) {
            keys.push_back(entry.second); // Add the value to the vector
        }
    }
} // namespace mrover


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mrover::ImageKeyDetector)

#include "key_detector.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
namespace mrover {
    auto KeyDetectorBase::preprocessYOLOv8Input(Model const& model, cv::Mat const& input, cv::Mat& output) -> void {
        static cv::Mat bgrImage;
        static cv::Mat blobSizedImage;

        cv::cvtColor(input, bgrImage, cv::COLOR_BGRA2BGR);

        if (model.inputTensorSize.size() != 4) {
            throw std::runtime_error("Expected Blob Size to be of size 4, are you using the correct model type?");
        }

        static constexpr double UCHAR_TO_DOUBLE = 1.0 / 255.0;

        cv::Size blobSize{static_cast<int32_t>(model.inputTensorSize[2]), static_cast<int32_t>(model.inputTensorSize[3])};
        cv::resize(bgrImage, blobSizedImage, blobSize);
        cv::dnn::blobFromImage(blobSizedImage, output, UCHAR_TO_DOUBLE, blobSize, cv::Scalar{}, false, false);
    }

    auto KeyDetectorBase::postprocessYOLOv8Output(Model const& model, cv::Mat& output) -> void {
        // All processing should be done in parseYOLOv8Output
    }

    auto KeyDetectorBase::preprocessTextCoordsInput(Model const& model, cv::Mat const& input, cv::Mat& output) -> void {
        static cv::Mat bgrImage;
        
        // Convert to correct color scheme
        cv::cvtColor(input, bgrImage, cv::COLOR_BGRA2BGR);

        std::filesystem::path packagePath = std::filesystem::path{ament_index_cpp::get_package_prefix("mrover")} / ".." / ".." / "src" / "mrover" / "perception" / "key_detector" / "datasets" / "test" / "image" / "f91.png" ;

        bgrImage = cv::imread(packagePath.c_str(), cv::IMREAD_COLOR);

        static constexpr float SCALE_FACTOR = 1.0f;
        static constexpr std::size_t CHANNELS = 3;
        cv::Size blobSize{static_cast<int32_t>(model.inputTensorSize[2]), static_cast<int32_t>(model.inputTensorSize[3])};
        cv::dnn::blobFromImage(bgrImage, output, SCALE_FACTOR, blobSize, cv::Scalar{}, false, false);

        // Normalize based on these values
        const cv::Vec3f targetMeans{100.34723, 100.90665, 95.4391};
        const cv::Vec3f targetStdDevs{78.34715, 76.304565, 77.18236};

        for(std::size_t channel = 0; channel < CHANNELS; ++channel){
            std::size_t numElts = static_cast<std::size_t>(blobSize.height * blobSize.width);
            std::size_t offset = channel * numElts;

            for(std::size_t elt = 0; elt < numElts; ++elt){
                float& v = reinterpret_cast<float*>(output.data)[offset + elt];
                v = (v - targetMeans.val[channel]) / targetStdDevs.val[channel];
            }
        }
    }

    auto KeyDetectorBase::postprocessTextCoordsOutput(Model const& model, cv::Mat& output) -> void {
        // assert(output.total == 8);

        // De-Normalize the output

        static constexpr std::array<float, 8> means{121.73, 304.18, 521.91, 321.19, 518.25, 182.49, 135.07, 166.05};
        static constexpr std::array<float, 8> stdDevs{295.45, 322.01, 291.26, 333.85, 291.21, 329.92, 298.24, 317.73};

        auto* data = reinterpret_cast<float*>(output.data);

        for(std::size_t i = 0; i < 8; ++i){
            float& v = data[i];
            v = v * stdDevs[i] + means[i];
        }

        R2f p1, p2, p3, p4;
        p1 << data[0], data[1];
        p2 << data[2], data[3];
        p3 << data[4], data[5];
        p4 << data[6], data[7];

        R2f u1, u2, u3, u4;
        u1 << 0, 0;
        u2 << 1, 0;
        u3 << 1, 1;
        u4 << 0, 1;

        auto xyBasis = changeOfBasis(p1, p2, p3, p4);
        auto uvBasis = changeOfBasis(u1, u2, u3, u4);

        auto C = uvBasis * xyBasis.inverse();

        static constexpr float RESOLUTION = 2.0f;
        static constexpr float WIDTH = 640;
        static constexpr float HEIGHT = 480;

        if(output.cols != static_cast<int>(WIDTH/RESOLUTION) || output.rows != static_cast<int>(HEIGHT/RESOLUTION))
            output.create(static_cast<int>(HEIGHT/RESOLUTION), static_cast<int>(WIDTH/RESOLUTION), CV_32FC3);
        auto* coords = reinterpret_cast<R3f*>(output.data);
        std::size_t index = 0;

        for(float y = 0.0f; y < HEIGHT;){
            y += RESOLUTION; 
            for(float x = 0.0f; x < WIDTH;){
                x += RESOLUTION;
                R3f& coord = coords[index];

                coord << x, y, 1.0f;

                coord = C * coord;

                coord.x() /= coord.z();
                coord.y() /= coord.z();
                coord.z() /= coord.z();

                ++index;
            }
        }

        // Create an image from the keyboard gradient
        // cv::Mat temp;
        // output.convertTo(temp, CV_8UC3, 255.0);
        // cv::cvtColor(temp, output, cv::COLOR_BGR2BGRA);
    }

    auto KeyDetectorBase::parseYOLOv8Output(Model const& model, cv::Mat& output, std::vector<Detection>& detections) const -> void {
        // Parse model specific dimensioning from the output
        // The input to this function is expecting a YOLOv8 style model, thus the dimensions should be > rows
        if (output.cols <= output.rows) {
            std::array<char, 75> message{};
            snprintf(message.data(), message.size(), "Something is wrong model interpretation, %d cols and %d rows", output.cols, output.rows);
            throw std::runtime_error(message.data());
        }

        int numCandidates = output.cols;
        int predictionDimension = output.rows;

        // The output of the model is a batchSizex84x8400 matrix
        // This converts the model to a TODO: Check this dimensioning
        output = output.reshape(1, predictionDimension);
        cv::transpose(output, output);

        // This function expects the image to already be in the correct format thus no distrotion is needed
        float const xFactor = 1.0;
        float const yFactor = 1.0;

        // Intermediate Storage Containers
        std::vector<int> classIds;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;

        // Each row of the output is a detection with a bounding box and associated class scores
        for (int r = 0; r < numCandidates; ++r) {
            // Skip first four values as they are the box data
            cv::Mat scores = output.row(r).colRange(4, predictionDimension);

            cv::Point classId;
            double maxClassScore;
            cv::minMaxLoc(scores, nullptr, &maxClassScore, nullptr, &classId);

            // Determine if the class is an acceptable confidence level, otherwise disregard
            if (maxClassScore <= mModelScoreThreshold) continue;

            confidences.push_back(static_cast<float>(maxClassScore));
            classIds.push_back(classId.x);

            // Get the bounding box data
            cv::Mat box = output.row(r).colRange(0, 4);
            auto x = box.at<float>(0);
            auto y = box.at<float>(1);
            auto w = box.at<float>(2);
            auto h = box.at<float>(3);

            // Cast the corners into integers to be used on pixels
            auto left = static_cast<int>((x - 0.5 * w) * xFactor);
            auto top = static_cast<int>((y - 0.5 * h) * yFactor);
            auto width = static_cast<int>(w * xFactor);
            auto height = static_cast<int>(h * yFactor);

            // Push abck the box into storage
            boxes.emplace_back(left, top, width, height);
        }

        //Coalesce the boxes into a smaller number of distinct boxes
        std::vector<int> nmsResult;
        cv::dnn::NMSBoxes(boxes, confidences, mModelScoreThreshold, mModelNMSThreshold, nmsResult);

        // Fill in the output Detections Vector
        for (int i: nmsResult) {
            //Push back the detection into the for storage vector
            detections.emplace_back(classIds[i], model.classes[classIds[i]], confidences[i], boxes[i]);
        }
    }

    auto KeyDetectorBase::updateMedians(std::priority_queue<float>& left, std::priority_queue<float, std::vector<float>, std::greater<>>& right, float val) -> void{
            // Insert new element into max heap
            left.push(val);
            
            // Move the top of max heap to min heap to maintain order
            float temp = left.top();
            left.pop();
            right.push(temp);
          
            // Balance heaps if min heap has more elements
            if (right.size() > left.size()) {
                temp = right.top();
                right.pop();
                left.push(temp);
            }
    }

    auto KeyDetectorBase::matchKeyDetections(cv::Mat const& gradient, std::vector<Detection> const& _detections) -> void{
        // fake data
        // TODO: remove
        std::vector<Detection> detections;
        detections.emplace_back(0, "test0", 0.4 ,cv::Rect(111, 222, 20, 30));
        detections.emplace_back(0, "test1", 0.2 ,cv::Rect(222, 111, 32, 12));

        // resize the mask to be 640 by 640
        static cv::Mat resizedGradient;
        cv::resize(gradient, resizedGradient, cv::Size{256, 256});

        // For each of the bounding boxes, find the median red and blue
        std::vector<cv::Vec2f> medians;
        medians.reserve(detections.size());
        int index = 0;
        for(auto const& detect : detections){
            int x = detect.box.x;
            int y = detect.box.y;
            int width = detect.box.width;
            int height = detect.box.height;

            std::priority_queue<float> blueMaxHeap;
            std::priority_queue<float, std::vector<float>, std::greater<>> blueMinHeap;

            std::priority_queue<float> greenMaxHeap;
            std::priority_queue<float, std::vector<float>, std::greater<>> greenMinHeap;

            // Streaming Median Algorithm: See https://www.geeksforgeeks.org/median-of-stream-of-integers-running-integers/
            for(int h = 0; h < height; ++h){
                for(int w = 0; w < width; ++w){
                    auto const& v = resizedGradient.at<cv::Vec3f>(y + h, x + w);

                    // Update each pq
                    updateMedians(blueMaxHeap, blueMinHeap, v.val[0]);
                    updateMedians(greenMaxHeap, greenMinHeap, v.val[1]);
                }
            }

            auto findMedian = [](std::priority_queue<float>& left, std::priority_queue<float, std::vector<float>, std::greater<>>& right){
                float median;
                if (left.size() != right.size())
                    median = left.top();
                else
                    median = static_cast<float>((left.top() + right.top()) / 2);
                return median;
            };

            float blueMedian = 255 * findMedian(blueMaxHeap, blueMinHeap);
            float greenMedian = 255 * findMedian(greenMaxHeap, greenMinHeap);

            RCLCPP_INFO_STREAM(get_logger(), ++index << " BRO " << blueMedian << " " << greenMedian);

            medians.emplace_back(blueMedian, greenMedian);
        }

        static const std::array<cv::Vec2f, 87> colors {
            cv::Vec2f(43, 27),
            cv::Vec2f(91, 32),
            cv::Vec2f(139, 27),
            cv::Vec2f(157, 27),
            cv::Vec2f(175, 27),
            cv::Vec2f(193, 27),
            cv::Vec2f(215, 27),
            cv::Vec2f(48, 66),
            cv::Vec2f(62, 66),
            cv::Vec2f(228, 27),
            cv::Vec2f(75, 66),
            cv::Vec2f(241, 27),
            cv::Vec2f(89, 66),
            cv::Vec2f(103, 66),
            cv::Vec2f(117, 66),
            cv::Vec2f(130, 66),
            cv::Vec2f(144, 66),
            cv::Vec2f(158, 66),
            cv::Vec2f(41, 107),
            cv::Vec2f(183, 67),
            cv::Vec2f(54, 107),
            cv::Vec2f(229, 66),
            cv::Vec2f(68, 107),
            cv::Vec2f(82, 107),
            cv::Vec2f(96, 107),
            cv::Vec2f(110, 107),
            cv::Vec2f(124, 107),
            cv::Vec2f(137, 107),
            cv::Vec2f(151, 107),
            cv::Vec2f(37, 148),
            cv::Vec2f(165, 107),
            cv::Vec2f(51, 148),
            cv::Vec2f(65, 148),
            cv::Vec2f(186, 107),
            cv::Vec2f(78, 148),
            cv::Vec2f(92, 148),
            cv::Vec2f(106, 148),
            cv::Vec2f(120, 148),
            cv::Vec2f(134, 148),
            cv::Vec2f(46, 187),
            cv::Vec2f(147, 148),
            cv::Vec2f(161, 148),
            cv::Vec2f(60, 187),
            cv::Vec2f(175, 148),
            cv::Vec2f(73, 187),
            cv::Vec2f(192, 148),
            cv::Vec2f(87, 187),
            cv::Vec2f(228, 151),
            cv::Vec2f(214, 151),
            cv::Vec2f(241, 151),
            cv::Vec2f(101, 187),
            cv::Vec2f(115, 187),
            cv::Vec2f(128, 188),
            cv::Vec2f(32, 231),
            cv::Vec2f(142, 188),
            cv::Vec2f(45, 231),
            cv::Vec2f(156, 187),
            cv::Vec2f(59, 231),
            cv::Vec2f(169, 187),
            cv::Vec2f(73, 231),
            cv::Vec2f(189, 187),
            cv::Vec2f(241, 188),
            cv::Vec2f(228, 188),
            cv::Vec2f(215, 188),
            cv::Vec2f(94, 231),
            cv::Vec2f(107, 231),
            cv::Vec2f(120, 231),
            cv::Vec2f(133, 231),
            cv::Vec2f(155, 231),
            cv::Vec2f(168, 231),
            cv::Vec2f(182, 231),
            cv::Vec2f(195, 230),
            cv::Vec2f(241, 231),
            cv::Vec2f(214, 231),
            cv::Vec2f(228, 231),
            cv::Vec2f(7, 27),
            cv::Vec2f(24, 27),
            cv::Vec2f(13, 66),
            cv::Vec2f(34, 66),
            cv::Vec2f(9, 107),
            cv::Vec2f(28, 107),
            cv::Vec2f(23, 148),
            cv::Vec2f(32, 187),
            cv::Vec2f(7, 148),
            cv::Vec2f(18, 187),
            cv::Vec2f(5, 187),
            cv::Vec2f(5, 230)
        };

        // create distance matrix between each of the detection and every known key
        // 0 - weight between 1st source & 1st target (W11)
        // 1 - weight between 1st source & 2nd target (W12)
        // m - weight between 1st source & mth target (W1m)
        // ...
        Eigen::VectorXf weights(2 * medians.size() * colors.size());
        for(std::size_t n = 0; n < medians.size(); ++n){
            for(std::size_t m = 0; m < colors.size(); ++m){
                auto norm = [](cv::Vec2f const& lhs, cv::Vec2f const& rhs) -> float{
                    return static_cast<float>(std::sqrt(std::pow(lhs.val[0] - rhs.val[0], 2) + std::pow(lhs.val[1] - rhs.val[1], 2)));
                };

                float val = static_cast<float>(1e6 / std::pow(norm(medians[n], colors[m]), 3));

                weights(static_cast<int>(n * colors.size() + m)) = val;
                weights(static_cast<int>(colors.size() * medians.size() + n * colors.size() + m)) = val;
            }
        }

        static Eigen::VectorXf prevWeights = weights;

        RCLCPP_INFO_STREAM(get_logger(), "Weights equality " << (prevWeights == weights));

        prevWeights = weights;

        // create targets vector
        // flat array of n of the colors vectors augmented together
        Eigen::VectorXf targets(2 * medians.size() * colors.size());

        //                                vvv num indices into vec2f
        for(std::size_t index = 0; index < 2; ++index){
            for(std::size_t i = 0; i < medians.size(); ++i){
                for(std::size_t ii = 0; ii < colors.size(); ++ii){
                    targets(static_cast<int>(index * medians.size() * colors.size() + i * colors.size() + ii)) = weights(static_cast<int>(index * medians.size() * colors.size() + i * colors.size() + ii)) * colors[ii].val[index];
                }
            }
        }

        static Eigen::VectorXf prevTargets = targets;

        RCLCPP_INFO_STREAM(get_logger(), "Targets equality " << (prevTargets == targets));

        prevTargets = targets;

        // create the P matrix
        Eigen::MatrixXf P(2 * colors.size() * medians.size(), 6);
        for(std::size_t n = 0; n < medians.size(); ++n){
            for(std::size_t m = 0; m < colors.size(); ++m){
                int offset = static_cast<int>(n * colors.size() + m);
                P(offset, 0) = medians[n].val[0] * weights(offset);
                P(offset, 1) = medians[n].val[1] * weights(offset);
                P(offset, 2) = 0.0f;
                P(offset, 3) = 0.0f;
                P(offset, 4) = 1.0f * weights(offset);
                P(offset, 5) = 0.0f;
            }
        }

        for(std::size_t n = 0; n < medians.size(); ++n){
            for(std::size_t m = 0; m < colors.size(); ++m){
                int offset = static_cast<int>(colors.size() * medians.size() + n * colors.size() + m);
                P(offset, 0) = 0.0f;
                P(offset, 1) = 0.0f;
                P(offset, 2) = medians[n].val[0] * weights(offset);
                P(offset, 3) = medians[n].val[1] * weights(offset);
                P(offset, 4) = 0.0f;
                P(offset, 5) = 1.0f * weights(offset);
            }
        }

        static Eigen::MatrixXf prevP = P;

        RCLCPP_INFO_STREAM(get_logger(), "P equality " << (prevP == P));

        prevP = P;

        // find the least sqaures solution

        Eigen::MatrixXd A(3, 2);
        Eigen::VectorXd b(3);
        
        // Fixed input for testing
        A << 1, 2,
             3, 4,
             5, 6;
        b << 7, 8, 9;
        
        for (int i = 0; i < 10; ++i) {
            // Ensure that A and b are not modified unexpectedly between iterations
            Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);
            Eigen::VectorXf y = P.colPivHouseholderQr().solve(targets);

            RCLCPP_INFO_STREAM(get_logger(), "Iteration " << i << " solution: " << y.transpose());
            // Print the solution for each iteration
            RCLCPP_INFO_STREAM(get_logger(), "Iteration " << i << " solution: " << x.transpose());
        }
    }

    auto KeyDetectorBase::changeOfBasis(R2f const& p1, R2f const& p2, R2f const& p3, R2f const& p4) -> Eigen::Matrix3f{
        // See https://math.stackexchange.com/questions/296794/finding-the-transform-matrix-from-4-projected-points-with-javascript

        /*
         * X = [x4, y4, 1]
         */

        R3f X;
        X << p4.x(), p4.y(), 1;

        /*
         * M = [[x1, x2, x3],
         *      [y1, y2, y3],
         *      [ 1,  1,  1]]
         */
        Eigen::Matrix3f M;
        M << p1.x(), p2.x(), p3.x(),
             p1.y(), p2.y(), p3.y(),
             1.0f,   1.0f,   1.0f;

        Eigen::Matrix3f invM = M.inverse();

        /*
         * H = M @ X = [λ, μ, τ]
         */

        R3f H = invM * X;

        /*
         * A = [[λ * x1, μ * x2, τ * x3],
         *      [λ * y1, μ * y2, τ * y3],
         *      [     λ,      μ,     τ]]
         */

        Eigen::Matrix3f A;
        A <<    H(0) * p1.x(), H(1) * p2.x(), H(2) * p3.x(),
                H(0) * p1.y(), H(1) * p2.y(), H(2) * p3.y(),
                H(0),          H(1),          H(2);

        return A;
    }
}; // namespace mrover

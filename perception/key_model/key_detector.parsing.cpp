#include "key_detector.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cassert>
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

        // std::filesystem::path packagePath = std::filesystem::path{ament_index_cpp::get_package_prefix("mrover")} / ".." / ".." / "src" / "mrover" / "perception" / "key_detector" / "datasets" / "test" / "image" / "f91.png" ;

        // bgrImage = cv::imread(packagePath.c_str(), cv::IMREAD_COLOR);

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

        static constexpr float XRESOLUTION = 1.0f;
        static constexpr float YRESOLUTION = 0.66f;
        static constexpr float WIDTH = 640;
        static constexpr float HEIGHT = 480;

        if(output.cols != static_cast<int>(WIDTH/XRESOLUTION) || output.rows != static_cast<int>(HEIGHT/XRESOLUTION))
            output.create(static_cast<int>(HEIGHT/XRESOLUTION), static_cast<int>(WIDTH/XRESOLUTION), CV_32FC3);
        auto* coords = reinterpret_cast<R3f*>(output.data);
        std::size_t index = 0;


        for(float y = 0.0f; y < HEIGHT;){
            y += XRESOLUTION; 
            for(float x = 0.0f; x < WIDTH;){
                x += XRESOLUTION;
                R3f& coord = coords[index];

                coord << x, YRESOLUTION * y, 1.0f;

                coord = C * coord;

                coord.x() /= coord.z();
                coord.y() /= coord.z();
                coord.z() /= coord.z();

                ++index;
            }
        }

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

            // Get the bounding box data
            cv::Mat box = output.row(r).colRange(0, 4);
            auto x = box.at<float>(0);
            auto y = box.at<float>(1);
            auto w = box.at<float>(2);
            auto h = box.at<float>(3);

            // Cast the corners into integers to be used on pixels
            auto left = std::min(std::max(static_cast<int>((x - 0.5 * w) * xFactor), 0), 640 - 1);
            auto top = std::min(std::max(static_cast<int>((y - 0.5 * h) * yFactor), 0), 480 - 1);
            auto width = std::min(static_cast<int>(w * xFactor), (640 - 1) - left);
            auto height = std::min(static_cast<int>(h * yFactor), (480 - 1) - top);
            
            // only push back if there is a width and height
            if(!width || !height)
                continue;

            // Push abck the box into storage
            boxes.emplace_back(left, top, width, height);
            confidences.push_back(static_cast<float>(maxClassScore));
            classIds.push_back(classId.x);
        }

        //Coalesce the boxes into a smaller number of distinct boxes
        std::vector<int> nmsResult;
        cv::dnn::NMSBoxes(boxes, confidences, mModelScoreThreshold, mModelNMSThreshold, nmsResult);

        // Fill in the output Detections Vector
        for (int i: nmsResult) {
            //Push back the detection into the for storage vector
            if(detections.size() < 87)
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

    auto KeyDetectorBase::matchKeyDetections(cv::Mat const& gradient, std::vector<Detection>& detections) -> void{
        // assert that all bounding boxes are inside the image
        for(auto const& det : detections){
            RCLCPP_INFO_STREAM(get_logger(), "Detection X: " << det.box.x << " Y: " << det.box.y << " W: " << det.box.width << " H: " << det.box.height);
            assert(det.box.tl().x >= 0);
            assert(det.box.tl().y >= 0);
            assert(det.box.br().x < mParsingSize.width);
            assert(det.box.br().y < mParsingSize.height);
            assert(det.box.width);
            assert(det.box.height);
        }

        // assert the gradient is the expected size
        assert(gradient.rows == 480);
        assert(gradient.cols == 640);

        // For each of the bounding boxes, find the median red and blue
        std::vector<cv::Vec2f> medians;
        medians.reserve(detections.size());
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
                    auto const& v = gradient.at<cv::Vec3f>(y + h, x + w);

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

        static const std::array<std::string, 87> keyNames {
            "lalt",
            "space",
            "ralt",
            "fn",
            "menu",
            "rctrl",
            "left",
            "X",
            "C",
            "down",
            "V",
            "right",
            "B",
            "N",
            "M",
            "<",
            ">",
            "?",
            "S",
            "rshift",
            "D",
            "up",
            "F",
            "G",
            "H",
            "J",
            "K",
            "L",
            ";",
            "W",
            "\"",
            "E",
            "R",
            "enter",
            "T",
            "Y",
            "U",
            "I",
            "O",
            "3",
            "P",
            "[",
            "4",
            "]",
            "5",
            "|",
            "6",
            "end",
            "del",
            "pg dn",
            "7",
            "8",
            "9",
            "f1",
            "0",
            "f2",
            "-",
            "f3",
            "+",
            "f4",
            "back",
            "pg up",
            "home",
            "insert",
            "f5",
            "f6",
            "f7",
            "f8",
            "f9",
            "f10",
            "f11",
            "f12",
            "pause",
            "print",
            "lock",
            "lctrl",
            "windows",
            "lshift",
            "Z",
            "caps",
            "A",
            "Q",
            "2",
            "ltab",
            "1",
            "`",
            "esc"
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

        Eigen::VectorXf y = P.colPivHouseholderQr().solve(targets);
        float a = y(0), b = y(1), c = y(2), d = y(3), tx = y(4), ty = y(5);

        Eigen::Matrix3f A;
        A <<    a, b, tx,
                c, d, ty,
                0.0f, 0.0f, 1.0f;

        //                                        vvv this is dim(point) + 1
        Eigen::MatrixXf results(medians.size(), 3);
        for(std::size_t r = 0; r < medians.size(); ++r){
            Eigen::Vector3f v{medians[r].val[0], medians[r].val[1], 1.0f};
            Eigen::Vector3f row = A * v;
            for(std::size_t c = 0; c < 3; ++c){
                results(static_cast<int>(r), static_cast<int>(c)) = row(static_cast<int>(c));
            }
        }

        // create a cost matrix between all transformed points and the colors
        Eigen::MatrixXf cost(medians.size(), colors.size());
        for(std::size_t r = 0; r < medians.size(); ++r){
            for(std::size_t c = 0; c < colors.size(); ++c){
                cost(static_cast<int>(r), static_cast<int>(c)) = static_cast<float>(std::sqrt(std::pow(results(static_cast<int>(r), 0) - colors[c].val[0], 2) + std::pow(results(static_cast<int>(r), 1) - colors[c].val[1], 2)));
            }
        }

        auto keyAssignments = hungarian(cost);

        std::size_t index = 0;
        for(std::size_t k = 0; k < keyAssignments.size(); ++k){
            auto key = keyAssignments[k];
            RCLCPP_INFO_STREAM(get_logger(), "first " << key.first << " second " << key.second);
            if(key.first != -1){
                detections[key.first].className = keyNames[key.second];
                RCLCPP_INFO_STREAM(get_logger(), "Key " << detections[key.first].className);
                ++index;
            }
        }
    }

    template <class T>
    auto ckmin(T &a, const T &b) -> bool {
        return b < a ? a = b, 1 : 0;
    }

    auto KeyDetectorBase::hungarian(const Eigen::MatrixXf &C) -> std::vector<std::pair<int, int>> {
        const int J = static_cast<int>(C.rows()), W = static_cast<int>(C.cols());
        assert(J <= W);

        std::vector<int> job(W + 1, -1);
        std::vector<float> ys(J), yt(W + 1);  // potentials
        const float inf = std::numeric_limits<float>::max();
        for (int j_cur = 0; j_cur < J; ++j_cur) {  // assign j_cur-th job
            int w_cur = W;
            job[w_cur] = j_cur;

            std::vector<float> min_to(W + 1, inf);
            std::vector<int> prv(W + 1, -1);
            std::vector<bool> in_Z(W + 1);

            while (job[w_cur] != -1) {   // runs at most j_cur + 1 times
                in_Z[w_cur] = true;
                const int j = job[w_cur];
                float delta = inf;
                int w_next;
                for (int w = 0; w < W; ++w) {
                    if (!in_Z[w]) {
                        if (ckmin(min_to[w], C(j, w) - ys[j] - yt[w]))
                            prv[w] = w_cur;
                        if (ckmin(delta, min_to[w])) w_next = w;
                    }
                }
                for (int w = 0; w <= W; ++w) {
                    if (in_Z[w]) ys[job[w]] += delta, yt[w] -= delta;
                    else min_to[w] -= delta;
                }
                w_cur = w_next;
            }

            for (int w; w_cur != W; w_cur = w) job[w_cur] = job[w = prv[w_cur]];
        }

        std::vector<std::pair<int, int>> result;
        for (size_t i = 0; i < job.size() - 1; ++i) {
            result.emplace_back(job[i], i);
        }
        auto pairLess = [&](std::pair<int, int> const& lhs, std::pair<int, int> const& rhs){
            return lhs.first < rhs.first;
        };

        std::sort(std::begin(result), std::end(result), pairLess);

        return result;
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

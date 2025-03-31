#include "key_detector.hpp"
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

        std::filesystem::path packagePath = std::filesystem::path{ament_index_cpp::get_package_prefix("mrover")} / ".." / ".." / "src" / "mrover" / "perception" / "key_detector" / "datasets" / "test" / "image" / "f52.png" ;

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
        assert(output.total() == 8);

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


        // AT THIS POINT IT IS CORRECT

        // Create an image from the keyboard gradient
        cv::Mat temp;
        output.convertTo(temp, CV_8UC3, 255.0);
        cv::cvtColor(temp, output, cv::COLOR_BGR2BGRA);
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

    auto KeyDetectorBase::matchKeyDetections(cv::Mat const& gradient, std::vector<Detection> detections) -> void{
        
        // For each of the bounding boxes, find the median red and blue
        int index = 0;
        //std::vector<cv::Point2d> source;

        cv::Mat source(detections.size(), 2, 1);
        //source.reserve(detections.size());
        for(int i  = 0; i < detections.size(); i++){
            auto detect = detections[i];
            int x = detect.box.x;
            int y = detect.box.y;
            int width = detect.box.width;
            int height = detect.box.height;

            std::priority_queue<float> blueMaxHeap;
            std::priority_queue<float, std::vector<float>, std::greater<>> blueMinHeap;

            std::priority_queue<float> greenMaxHeap;
            std::priority_queue<float, std::vector<float>, std::greater<>> greenMinHeap;

            std::priority_queue<float> redMaxHeap;
            std::priority_queue<float, std::vector<float>, std::greater<>> redMinHeap;
            
            // Streaming Median Algorithm: See https://www.geeksforgeeks.org/median-of-stream-of-integers-running-integers/
            for(int h = 0; h < height; ++h){
                for(int w = 0; w < width; ++w){
                    auto const& v = gradient.at<cv::Vec3f>(y + h, x + w);
                    
                    // Update each pq
                    updateMedians(blueMaxHeap, blueMinHeap, v.val[0]);
                    updateMedians(greenMaxHeap, greenMinHeap, v.val[1]);
                    updateMedians(redMaxHeap, redMinHeap, v.val[2]);
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

            float blueMedian = findMedian(blueMaxHeap, blueMinHeap);
            float greenMedian = findMedian(greenMaxHeap, greenMinHeap);
            float redMedian = findMedian(redMaxHeap, redMinHeap);

            source.at<double>(i, 0) = redMedian;
            source.at<double>(i, 1) = greenMedian;

            RCLCPP_INFO_STREAM(get_logger(), ++index << " BRO " << blueMedian << " " << greenMedian << " " << redMedian);
        }

        auto n = source.rows;
        auto m = cv_palette.rows;

        auto create_distance_matrix = [](cv::Mat source, cv::Mat palette) -> auto {
            if (source.rows == 1)
            {
                return cv::Mat();
            }

            cv::Mat output = cv::Mat_(source.rows, palette.rows, 1);

            for (int i = 0; i < source.rows; ++i)
            {
                for (int j = 0; j < palette.rows; ++j)
                {
                    auto diff = source.at<double>(i) - palette.at<double>(j);

                    auto norm = cv::norm(diff, cv::NORM_L2);

                    output.at<double>(i,j) = norm;
                }
            }

            return output;
        };

        auto distance_matrix = create_distance_matrix(source, cv_palette);

        std::cout << cv::norm(distance_matrix, cv::NORM_L2) << '\n';

        if(distance_matrix.empty())
        {
            return;
        }

        cv::Mat tmp;
        pow(distance_matrix,1.5,tmp);
        
        cv::Mat distance_cost_matrix =  1e6 / tmp;

        cv::Mat W = distance_cost_matrix.reshape(1, distance_cost_matrix.total());

        W = cv::repeat(W, 1, 2);

        cv::Mat targets = cv::repeat(cv_palette, n, 1);
        
        // This badness is for Fortran style column based flattening
        cv::Mat targets_flat;
        cv::transpose(targets, targets_flat);
        targets_flat = targets_flat.reshape(1, targets_flat.total());

        targets = targets_flat * W;


        cv::Mat repeated_source = cv::repeat(source, 1, m);
        
        cv::Mat ones(n * m, 1, CV_32F, cv::Scalar(1));
        cv::Mat zeros(n * m, 1, CV_32F, cv::Scalar(0));

        cv::Mat p1;

        cv::hconcat(repeated_source, zeros, p1);
        cv::hconcat(p1, zeros, p1);
        cv::hconcat(p1, ones, p1);
        cv::hconcat(p1, zeros, p1);

        cv::Mat p2;

        cv::hconcat(zeros, zeros, p2);
        cv::hconcat(p2, repeated_source, p2);
        cv::hconcat(p2, zeros, p2);
        cv::hconcat(p2, ones, p2);

        cv::Mat p;

        cv::vconcat(p1, p2, p);

        cv::Mat W_column = W.reshape(1, W.total());

        p = p * W_column;

        cv::Mat output; 
        
        bool success = cv::solve(p, targets, output, cv::DECOMP_SVD);

        if (!success)
            return;

        std::vector<std::vector<float>> data = {
            {output.at<float>(0), output.at<float>(1), output.at<float>(2)},
            {output.at<float>(3), output.at<float>(4), output.at<float>(5)},
            {0, 0, 1}
        };
        cv::Mat A = cv::Mat(data);

        cv::Mat stacked;
        cv::hconcat(source, ones, stacked);

        cv::Mat result(A.rows, stacked.rows, CV_32F);

        for (int i = 0; i < A.rows; i++)
        {
            for (int j = 0; j < stacked.rows; j++)
            {
                result.at<float>(i,j) = A.row(i).dot(stacked.row(j));
            }
        }

        cv::Mat cost_matrix = create_distance_matrix(result, cv_palette);
        
        std::vector<int> col_ind;

        std::vector<std::string> key_results;

        for (int i = 0; i < detections.size(); i++)
        {
           detections[i].classId = col_ind[i];
           detections[i].className = keys[col_ind[i]];
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

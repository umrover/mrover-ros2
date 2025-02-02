#include "pch.hpp"
#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <rclcpp/executors.hpp>
#include <string>

namespace mrover{
    class pcdNode final : public rclcpp::Node{
        public: 

        pcdNode() : Node("pcd_generator"){}

        // Create subscriber and subscribe to /zed/left/points topic
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mPcSub = create_subscription<sensor_msgs::msg::PointCloud2>("/zed/left/points", 1, [this](sensor_msgs::msg::PointCloud2::UniquePtr const& msg) {
            pointCloudCallback(msg);
        });

        // Path to save files to
        std::string file = "perception/utils/baja1-";

        // File number of the message (each message gets a pcd file) will look like bajaX-X.pcd
        int msgNum = 1;

        void pointCloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr const& msg) {
            if(msgNum > 1){ return; } // Just a test to see if it works creating a single file
            // std::cout << "HERE1\n";

            std::ofstream outf(file + std::to_string(msgNum) + ".pcd");
            // std::cout << "HERE2\n";

            // Print the header to the file
            if(outf.is_open()){
                // File Header
                outf << "# .PCD v0.7 - Point Cloud Data file format\n";
                outf << "VERSION 0.7\n" << "FIELDS x y z normal_x normal_y normal_z rgb curvature\n";

                // Float, each of size 4 bytes, 1 of each
                outf << "SIZE 4 4 4 4 4 4 4 4\n" << "TYPE F F F F F F U F\n" << "COUNT 1 1 1 1 1 1 1 1\n";
                outf << "WIDTH " << msg->width * msg->height << "\n" << "HEIGHT 1\n";
                outf << "VIEWPOINT 0 0 0 1 0 0 0\n" << "POINTS " << msg->width * msg->height << "\n" << "DATA ascii\n";
                // std::cout << "WIDTH: " << msg->width << " HEIGHT: " << msg->height << "\n";
            }

            // Put message data in file
            // Must reinterpret cast because pc data is compact and just a bunch of bits before casting
            //      returns a pointer to the start of a point vector
            auto* points = reinterpret_cast<Point const*>(msg->data.data());

            // height = #rows, width = #cols, loop through, do some pointer math
            //      point vector is 1D under the hood, so r*#cols + c gives position in the 1D vector
            for(size_t r = 0; r < msg->height; r++){
                for(size_t c = 0; c < msg->width; c++){
                    Point const& p = points[r * msg->width + c];

                    // remove any points that have inf or nan x,y, or z (suffices to just check x coordinate)
                    if(std::isinf(p.x) || std::isnan(p.x)){
                        outf << "0 0 0 0 0 0 0 0\n";
                        continue;
                    }

                    // Print values
                    uint32_t rgb = ((uint32_t)p.r << 16 | (uint32_t)p.g << 8 | (uint32_t)p.b);
                    outf << p.x << " " << p.y << " " << p.z << " " << p.normal_x << " " << p.normal_y 
                         << " " << p.normal_z << " " << rgb << " " << p.curvature << "\n";
                }
            }

            outf.close();
            std::cout << "PCD " << file + std::to_string(msgNum) << ".pcd created\n";
            msgNum++;
        }
    };
} // namespace mrover

auto main(int argc, char** argv) -> int{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::pcdNode>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
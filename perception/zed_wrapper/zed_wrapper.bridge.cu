// Be careful what you include in this file, it is compiled with nvcc (NVIDIA CUDA compiler)

#include "point.hpp"

#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <sl/Camera.hpp>
#include <thrust/device_vector.h>

namespace mrover {

    using PointCloudGpu = thrust::device_vector<Point>;

    // Optimal for the Jetson Xavier NX - this is max threads per block and each block has a max of 2048 threads
    constexpr uint BLOCK_SIZE = 1024;

    /**
     * @brief Runs on the GPU, interleaving the XYZ and BGRA buffers into a single buffer of #Point structs.
     */
    __global__ void fillPointCloudMessageKernel(sl::float4* xyzGpuPtr, sl::uchar4* bgraGpuPtr, sl::float4* normalsGpuPtr, Point* pcGpuPtr, size_t size) {
        // This function is invoked once per element at index #i in the point cloud
        size_t const i = blockIdx.x * blockDim.x + threadIdx.x;
        if (i >= size) return;

        pcGpuPtr[i].x = xyzGpuPtr[i].x;
        pcGpuPtr[i].y = xyzGpuPtr[i].y;
        pcGpuPtr[i].z = xyzGpuPtr[i].z;
        pcGpuPtr[i].b = bgraGpuPtr[i].r;
        pcGpuPtr[i].g = bgraGpuPtr[i].g;
        pcGpuPtr[i].r = bgraGpuPtr[i].b;
        pcGpuPtr[i].a = bgraGpuPtr[i].a;
        // pcGpuPtr[i].normal_x = normalsGpuPtr[i].x;
        // pcGpuPtr[i].normal_y = normalsGpuPtr[i].y;
        // pcGpuPtr[i].normal_z = normalsGpuPtr[i].z;
    }

    void checkCudaError(cudaError_t err) {
        if (err == cudaSuccess) return;

        //RCLCPP_ERROR_STREAM(rclcpp::get_logger("cuda_error"), "CUDA error: " << cudaGetErrorString(err));
        throw std::runtime_error("CUDA error");
    }

    /**
     * Fills a PointCloud2 message residing on the CPU from two GPU buffers (one for XYZ and one for BGRA).
     *
     * @param xyzGpu    XYZ buffer on the GPU
     * @param bgraGpu   BGRA buffer on the GPU
     * @param pcGpu     Point cloud buffer on the GPU (@see Point)
     * @param msg       Point cloud message with buffer on the CPU
     */
    void fillPointCloudMessageFromGpu(sl::Mat& xyzGpu, sl::Mat& bgraGpu, sl::Mat& normalsGpu, PointCloudGpu& pcGpu, sensor_msgs::msg::PointCloud2::UniquePtr const& msg) {
        assert(bgraGpu.getWidth() >= xyzGpu.getWidth());
        assert(bgraGpu.getHeight() >= xyzGpu.getHeight());
        assert(bgraGpu.getChannels() == 4);
        assert(xyzGpu.getChannels() == 4); // Last channel is unused
        assert(msg);

        auto* bgraGpuPtr = bgraGpu.getPtr<sl::uchar4>(sl::MEM::GPU);
        auto* xyzGpuPtr = xyzGpu.getPtr<sl::float4>(sl::MEM::GPU);
        auto* normalsGpuPtr = normalsGpu.getPtr<sl::float4>(sl::MEM::GPU);
        msg->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        msg->is_dense = true;
        msg->height = bgraGpu.getHeight();
        msg->width = bgraGpu.getWidth();
        fillPointCloudMessageHeader(msg);
        std::size_t size = msg->width * msg->height;

        pcGpu.resize(size);
        Point* pcGpuPtr = pcGpu.data().get();
        dim3 threadsPerBlock{BLOCK_SIZE};
        dim3 numBlocks{static_cast<uint>(std::ceil(static_cast<float>(size) / BLOCK_SIZE))};
        fillPointCloudMessageKernel<<<numBlocks, threadsPerBlock>>>(xyzGpuPtr, bgraGpuPtr, normalsGpuPtr, pcGpuPtr, size);
        checkCudaError(cudaPeekAtLastError());
        checkCudaError(cudaMemcpy(msg->data.data(), pcGpuPtr, size * sizeof(Point), cudaMemcpyDeviceToHost));
    }


} // namespace mrover

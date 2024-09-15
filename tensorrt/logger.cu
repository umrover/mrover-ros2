#include "logger.cuh"

namespace nvinfer1 {

    auto Logger::log(Severity severity, char const* msg) noexcept -> void {
        switch (severity) {
            case Severity::kINTERNAL_ERROR:
				std::cout << "[FATAL] " << msg << "\n";
                break;
            case Severity::kERROR:
				std::cout << "[ERROR] " << msg << "\n";
                break;
            case Severity::kWARNING:
				std::cout << "[WARN] " << msg << "\n";
                break;
            case Severity::kINFO:
				std::cout << "[INFO] " << msg << "\n";
                break;
            case Severity::kVERBOSE:
				std::cout << "[VERBOSE] " << msg << "\n";
                break;
        }
    }

} // namespace nvinfer1

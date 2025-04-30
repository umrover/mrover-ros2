#pragma once

#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include <gst.hpp>

namespace mrover::gst {

    class PipelineBuilder {

        std::vector<std::string> mElements;

    public:
        PipelineBuilder() = default;

        auto pushBack(std::string_view elem) -> PipelineBuilder& {
            mElements.emplace_back(elem);
            return *this;
        }

        template<typename... Args>
        auto pushBack(std::string_view elem, Args&&... props) -> PipelineBuilder&
            requires AllConvertibleToStringView<Args...>
        {
            std::ostringstream oss;
            oss << elem;
            ((oss << ' ' << std::forward<Args>(props)), ...);
            mElements.emplace_back(oss.str());
            return *this;
        }

        auto popBack() -> PipelineBuilder& {
            if (!mElements.empty()) {
                mElements.pop_back();
            }
            return *this;
        }

        auto insert(std::size_t index, std::string_view elem) -> PipelineBuilder& {
            if (index <= mElements.size()) {
                mElements.insert(mElements.begin() + static_cast<std::ptrdiff_t>(index), std::string(elem));
            }
            return *this;
        }

        template<typename... Args>
        auto insert(std::size_t index, std::string_view elem, Args&&... props) -> PipelineBuilder&
            requires AllConvertibleToStringView<Args...>
        {
            if (index <= mElements.size()) {
                std::ostringstream oss;
                oss << elem;
                ((oss << ' ' << std::forward<Args>(props)), ...);
                mElements.insert(mElements.begin() + static_cast<std::ptrdiff_t>(index), oss.str());
            }
            return *this;
        }

        auto remove(std::size_t index) -> PipelineBuilder& {
            if (index < mElements.size()) {
                mElements.erase(mElements.begin() + static_cast<std::ptrdiff_t>(index));
            }
            return *this;
        }

        auto clear() -> PipelineBuilder& {
            mElements.clear();
            return *this;
        }

        [[nodiscard]] auto size() const -> std::size_t {
            return mElements.size();
        }

        template<typename... Args>
        auto addPropsToElement(std::size_t index, Args&&... props) -> PipelineBuilder&
            requires AllConvertibleToStringView<Args...>
        {
            if (index < mElements.size()) {
                std::ostringstream oss;
                oss << mElements[index];
                ((oss << ' ' << std::forward<Args>(props)), ...);
                mElements[index] = oss.str();
            }
            return *this;
        }

        [[nodiscard]] auto str() const -> std::string {
            std::ostringstream oss;
            for (std::size_t i = 0; i < mElements.size(); ++i) {
                oss << mElements[i];
                if (i != mElements.size() - 1) {
                    oss << " ! ";
                }
            }
            return oss.str();
        }
    };

} // namespace mrover::gst

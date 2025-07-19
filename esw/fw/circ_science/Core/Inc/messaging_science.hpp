#pragma once

#include <array>
#include <cstdint>
#include <variant>

#include <units.hpp>

namespace mrover {

#pragma pack(push, 1)

    struct BaseScienceMessage {};

    struct SensorData {
		std::uint8_t id;
		double data;
	};

    enum class ScienceDataID : uint8_t {
		GEIGER = 1,
		HYDROGEN = 2,
		OZONE = 3,
	};

#pragma pack(pop)

} // namespace mrover

// Macros needed to operate on bitfields
#define SET_BIT_AT_INDEX(x, index, value) ((x) = ((x) & ~(1 << (index))) | ((value) << (index)))
#define GET_BIT_AT_INDEX(x, index) ((x) & (1 << (index)))

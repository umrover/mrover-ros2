#include "main.h"
#include "hardware_adc.hpp"

namespace mrover {
	class UVSensor {
	private:
		ADCSensor* adc_ptr;
		double uv_index;
		uint8_t channel;

	public:
		// adc_in is a pointer to the desired ADCSensor and channel_in is the channel that the UVsensor is on
		UVSensor (ADCSensor* adc_in, uint8_t channel_in)
			: adc_ptr(adc_in), uv_index(0), channel(channel_in) {
		};

		// update value of uv_index with blocking
		double update_uv_blocking() {
			int y = adc_ptr->get_raw_channel_blocking();
			float ratio = y/4096.0;
			uv_index = 33.0 * ratio;
			return uv_index;
		}

		// update value of uv_index with non-blocking
		double update_uv_async() {
			adc_ptr->update();
		    uv_index = adc_ptr->get_raw_channel_value(channel) / 100.00;
		    return uv_index;
		}

		// returns the current value of uv_index
		double get_current_uv() {
			return uv_index;
		}
	}; // class UVSensor
} // namespace mrover

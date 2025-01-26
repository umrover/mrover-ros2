#include "main.h"
#include "hardware_adc.hpp"
#include <math.h>

namespace mrover {
	class MethaneSensor {
	private:
		ADCSensor* adc_ptr;
		double ppm;
		uint8_t channel;

		float calculate_ppm(float analog_in) {
			double voltage = analog_in * 2.5 / 4096.00;
			double rs = (2.5 - voltage) / voltage;
			const double r0 = 0.92;
			ppm = powf(rs / r0, -3.20) * 860.00;
			return ppm;
		}

	public:
		// adc_in is a pointer to the desired ADCSensor and channel_in is the channel that the UVsensor is on
		MethaneSensor (ADCSensor* adc_in, uint8_t channel_in)
			: adc_ptr(adc_in), ppm(0), channel(channel_in) {
		};

		// update value in ppm with blocking
		double update_ppm_blocking() {
			ppm = calculate_ppm(adc_ptr->get_raw_channel_blocking());
			return ppm;
		}

		// update value in ppm with non-blocking
		double update_ppm_async() {
		    ppm = calculate_ppm(adc_ptr->get_raw_channel_value(channel));
		    return ppm;
		}

		// returns the current value in ppm
		double get_current_ppm() {
			return ppm;
		}
	}; // class UVSensor
} // namespace mrover

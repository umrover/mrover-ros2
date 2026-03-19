#include "main.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_def.h"

#define CO2_ADDR 0x29

namespace mrover {
	class CO2Sensor {
	private:
		I2C_HandleTypeDef* i2c; // i2c handle pointer
		uint8_t req_buf[2];
        uint8_t rx_buf[2];
		float percent; // ozone value in ppm

	public:
		CO2Sensor() = default;

		explicit CO2Sensor (I2C_HandleTypeDef* i2c_in)
			: i2c(i2c_in), req_buf{0x36, 0x39}, rx_buf{0x00, 0x00}, percent(0.0) {}

		// returns the current ozone data in ppm
		[[nodiscard]] float get_co2() const {
			return percent;
		}

        // updates ppm using rx_buf data -> conversion formula: ppm = ((rx_buf - 2^14) / 2^15) * 100
        float update_co2() {
            uint16_t raw = (rx_buf[0] << 8) | rx_buf[1];
            percent = (float(raw - (1 << 14)) / (1 << 15)) * 100.0f;
			
			return percent;
        }

        // requests raw co2 data over i2c, when sensor responds with data callback will be hit and will call receive_buf
		void request_co2() {
			HAL_I2C_Master_Transmit_IT(i2c, CO2_ADDR << 1, req_buf, 2);
		}

		// receives raw ozone data over i2c
		void receive_buf() {
			// on the first read sensor needs 100ms to update register
			static bool first_read = true;
			if (first_read) {
				HAL_Delay(100);
				first_read = false;
			}

			HAL_I2C_Master_Receive_IT(i2c, (CO2_ADDR << 1) | 1, rx_buf, 2);
		}

		// initializes sensor parameters
		bool init() {
            // Disable CRC (0x3768)
			uint8_t tx_buf1[2] = {0x37, 0x68};
            if (HAL_I2C_Master_Transmit(i2c, CO2_ADDR << 1, tx_buf1, 2, HAL_MAX_DELAY) != HAL_OK)
				return false;

            // Set measurement mode -> standard measurement mode with 0-25% concentration in air
			uint8_t tx_buf2[4] = {0x36, 0x15, 0x00, 0x11};
            if (HAL_I2C_Master_Transmit(i2c, CO2_ADDR << 1, tx_buf2, 4, HAL_MAX_DELAY) != HAL_OK)
				return false;

			return true;
		}
	};
}
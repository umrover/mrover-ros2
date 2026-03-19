#include "main.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_def.h"
#include <cstdint>

#define OZONE_ADDR	0x73 // i2c addr
#define MEASURE_MODE_AUTOMATIC	0x00 // auto mode
#define AUTO_DATA_HIGH_REGISTER	0x09 // auto mode data high byte
#define AUTO_DATA_LOW_REGISTER	0x0A // auto mode data low byte
#define MODE_REGISTER	0x03 // mode register

namespace mrover {
	class OzoneSensor {
	private:
		I2C_HandleTypeDef* i2c; // i2c handle pointer
		float ozone; // ozone value in ppm
		uint8_t rx_buf[2]; // receive buffer

	public:
		OzoneSensor() = default;

		OzoneSensor(I2C_HandleTypeDef* i2c_in)
			: i2c(i2c_in), ozone(0.0) {}

		// converts raw ozone data into ppm
		float update_ozone() {
			uint16_t ozone_raw = ((int16_t)rx_buf[0] << 8) | rx_buf[1];
			ozone = ozone_raw / 1000.0;
			
			return ozone;
		}

		// returns the current ozone data in ppm
		[[nodiscard]] float get_ozone() {
			return ozone;
		}

		// receives raw ozone data over i2c
		void read_ozone() {
			HAL_I2C_Mem_Read_IT(i2c, (OZONE_ADDR << 1) | 1, AUTO_DATA_HIGH_REGISTER, 1, rx_buf, 2);
		}

		// initializes the sensor to be in AUTO mode (sensor constantly sends data)
		bool init() {
			uint8_t mode = MEASURE_MODE_AUTOMATIC;
			if (HAL_I2C_Mem_Write(i2c, OZONE_ADDR << 1, MODE_REGISTER, 1, &mode, 1, HAL_MAX_DELAY) != HAL_OK)
				return false;

			return true;
		}
	};
}

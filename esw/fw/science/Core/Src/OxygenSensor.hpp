#include "main.h"
#include "hardware_adc.hpp"


#define ADDRESS_3 0x73
#define OXYGEN_DATA_REGISTER 0x03
#define OXYGEN_KEY_REGISTER 0x0A


namespace mrover {
	class OxygenSensor {
	private:
		uint8_t dev_addr;
		uint8_t mem_addr;
		uint8_t key_addr;
		I2C_HandleTypeDef *i2c;
		float calibration_multiplier;
		float percent;

	public:
		OxygenSensor (I2C_HandleTypeDef *i2c_in)
			: dev_addr(ADDRESS_3), key_addr(OXYGEN_KEY_REGISTER), mem_addr(OXYGEN_DATA_REGISTER), i2c(i2c_in), percent(0) {
			calibration_multiplier = (20.9 / 120.0); //default reasonable multiplier if calibration fails
		};

		float update_oxygen() {
			calibrate_oxygen();
			HAL_StatusTypeDef status;
			uint8_t buf[3];
			status = HAL_I2C_Mem_Read(i2c, dev_addr << 1, mem_addr, 1, buf, 3, 100);

			if (status != HAL_OK)
			{
				HAL_I2C_DeInit(i2c);
				HAL_Delay(5);
				HAL_I2C_Init(i2c);
				percent = -1;
				return percent;
			}

			percent = (calibration_multiplier * (((float)buf[0]) + ((float)buf[1] / 10.0) + ((float)buf[2] / 100.0))); // Could be that the 20.9/120.00 is not the correct value for _Key
			return percent;
		}

		float calibrate_oxygen() {
			uint8_t calibration_buf[1];
			HAL_StatusTypeDef status;
			status = HAL_I2C_Mem_Read(i2c, dev_addr << 1, key_addr, 1, calibration_buf, 1, 100);
			if (status != HAL_OK)
			{
				HAL_I2C_DeInit(i2c);
				HAL_Delay(5);
				HAL_I2C_Init(i2c);
				calibration_multiplier = (20.9 / 120.0);
			}
			else{
				calibration_multiplier = calibration_buf[0] / 1000.0;
			}
			return calibration_multiplier;
		}

		float get_oxygen() {
			return percent;
		}
	}; // class UVSensor
} // namespace mrover

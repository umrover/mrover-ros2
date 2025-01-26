#include "main.h"

#define TEMP_HUM_ADDRESS 0x38

namespace mrover {
	class TempHumiditySensor  {
	private:
		double temp;
		double humidity;
		uint8_t dev_addr;
		I2C_HandleTypeDef *i2c;

	public:
		TempHumiditySensor (I2C_HandleTypeDef *i2c_in)
			: temp(0), humidity(0), dev_addr(TEMP_HUM_ADDRESS), i2c(i2c_in){
		};

		void update_temp_humidity() {
			uint8_t buf[6];
			HAL_StatusTypeDef status;
			uint8_t command[3] = {0xAC, 0x33, 0x00};
			status = HAL_I2C_Master_Transmit(i2c, (dev_addr << 1), command, 3, 100);
			if (status != HAL_OK){
				temp = -1;
				humidity = -1;
				return;
			}

			HAL_Delay(100);

			status = HAL_I2C_Master_Receive(i2c, (dev_addr << 1) | 0x01, buf, 6, 100);
			if (status != HAL_OK){
				temp = -1;
				humidity = -1;
				return;
			}

			HAL_Delay(100);

			uint32_t SH = (buf[1] << 12) + (buf[2] << 4) + (buf[3] & 0b11110000);
			uint32_t ST = ((buf[3] & 0b00001111) << 16) + (buf[4] << 8) + buf[5];

			humidity = (SH / pow(2, 20)) * 100;
			temp = ((ST / pow(2, 20)) * 200) - 50;
		}

		double get_current_temp() {
			return temp;
		}

		double get_current_humidity() {
			return humidity;
		}
	}; // class TempHumiditySensor
} // namespace mrover

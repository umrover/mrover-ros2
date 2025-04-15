#include "main.h"
#include <limits>

#define TEMP_HUM_ADDRESS 0x38

namespace mrover {

	uint8_t th_buf[6];

    class TempHumiditySensor {
    private:
        double temp;
        double humidity;
        uint8_t dev_addr;
        I2C_HandleTypeDef* i2c;

    public:
        TempHumiditySensor() = default;

        TempHumiditySensor(I2C_HandleTypeDef* i2c_in)
            : temp(0), humidity(0), dev_addr(TEMP_HUM_ADDRESS), i2c(i2c_in) {
              };

        void update_temp_humidity() {
            HAL_StatusTypeDef status;
            uint8_t command[3] = {0xAC, 0x33, 0x00};
            status = HAL_I2C_Master_Transmit_IT(i2c, (dev_addr << 1), command, 3);
            if (status != HAL_OK) {
            	HAL_I2C_DeInit(i2c);
				HAL_I2C_Init(i2c);
                temp = std::numeric_limits<double>::quiet_NaN();
                humidity = std::numeric_limits<double>::quiet_NaN();
                return;
            }
        }

        void receive_buf() {
        	HAL_StatusTypeDef status;
        	status = HAL_I2C_Master_Receive_IT(i2c, (dev_addr << 1) | 0x01, th_buf, 6);
			if (status != HAL_OK) {
				HAL_I2C_DeInit(i2c);
				HAL_I2C_Init(i2c);
				temp = std::numeric_limits<double>::quiet_NaN();
				humidity = std::numeric_limits<double>::quiet_NaN();
				return;
			}
        }

        void calculate_th() {
        	uint32_t SH = (th_buf[1] << 12) + (th_buf[2] << 4) + (th_buf[3] & 0b11110000);
			uint32_t ST = ((th_buf[3] & 0b00001111) << 16) + (th_buf[4] << 8) + th_buf[5];

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

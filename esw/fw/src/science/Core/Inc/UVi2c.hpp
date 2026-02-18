#include "main.h"

#define UV_ADDRESS 0x53

namespace mrover {
    class UVSensor {
    private:
        double uv_index;
        uint8_t dev_addr;
        I2C_HandleTypeDef* i2c;

    public:
        UVSensor() = default;

        UVSensor(I2C_HandleTypeDef* i2c_in)
            : uv_index(0), dev_addr(UV_ADDRESS << 1), i2c(i2c_in) {
            };

        void init() {
			write_register(0x00, 0x0A); // enable uv mode
			write_register(0x04, 0x40); // set resolution to 16 bits and conversion time to 25 ms
        }

        void write_register(uint8_t reg, uint8_t value) {
            HAL_I2C_Mem_Write(i2c, dev_addr, reg, 1, &value, 1, 100);
        }

        uint8_t read_register(uint8_t reg) {
        	uint8_t uv_data = 0;
        	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(i2c, dev_addr, &reg, 1, 100);

        	if (status != HAL_OK) {
				HAL_I2C_DeInit(i2c);
				HAL_Delay(5);
				HAL_I2C_Init(i2c);
				return 0xFF;
			}

			status = HAL_I2C_Master_Receive(i2c, dev_addr, &uv_data, 1, 100);

			if (status != HAL_OK) {
				HAL_I2C_DeInit(i2c);
				HAL_Delay(5);
				HAL_I2C_Init(i2c);
				return 0xFF;
			}

        	return uv_data;
        }

        double update_uv() {
        	uint8_t byte0 = read_register(0x10);
        	uint8_t byte1 = read_register(0x11);
        	uint8_t byte2 = read_register(0x12) & 0x0F;
        	uv_index = (byte2 << 16) | (byte1 << 8) | byte0;
        	return uv_index;
        }

        double get_current_uv() {
            return uv_index;
        }
    }; // class TempHumiditySensor
} // namespace mrover

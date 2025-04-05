#include "hardware_adc.hpp"
#include <limits>

#define ADDRESS_3 0x73
#define OXYGEN_DATA_REGISTER 0x03
#define OXYGEN_KEY_REGISTER 0x0A

namespace mrover {

	uint8_t oxygen_buf[3];
	bool oxygen_state = 0; // 0 = calibration state, 1 = calculation state

    class OxygenSensor {
    private:
        uint8_t dev_addr;
        uint8_t mem_addr;
        uint8_t key_addr;
        I2C_HandleTypeDef* i2c;
        double calibration_multiplier;
        double percent;

    public:
        OxygenSensor() = default;

        OxygenSensor(I2C_HandleTypeDef* i2c_in)
            : dev_addr(ADDRESS_3), key_addr(OXYGEN_KEY_REGISTER), mem_addr(OXYGEN_DATA_REGISTER), i2c(i2c_in), percent(0) {
            calibration_multiplier = (20.9 / 120.0); //default reasonable multiplier if calibration fails
        };

        void update_oxygen() {
        	HAL_StatusTypeDef status;
			status = HAL_I2C_Mem_Read_IT(i2c, dev_addr << 1, key_addr, 1, oxygen_buf, 1);
			if (status != HAL_OK) {
				calibration_multiplier = std::numeric_limits<double>::quiet_NaN();
			}
        }

        void calibrate_oxygen() {
        	calibration_multiplier = (double) oxygen_buf[0] / 1000.0;
        	HAL_StatusTypeDef status;
			status = HAL_I2C_Mem_Read_IT(i2c, dev_addr << 1, mem_addr, 1, oxygen_buf, 3);
			if (status != HAL_OK) {
				calibration_multiplier = std::numeric_limits<double>::quiet_NaN();
				percent = std::numeric_limits<double>::quiet_NaN();
			}
        }

        void set_oxygen() {
        	if (calibration_multiplier == std::numeric_limits<double>::quiet_NaN())
				percent = std::numeric_limits<double>::quiet_NaN();
			else
				percent = calibration_multiplier * (((double) oxygen_buf[0]) + ((double) oxygen_buf[1] / 10.0) + ((double) oxygen_buf[2] / 100.0));
        }

        double get_oxygen() {
            return percent;
        }
    }; // class UVSensor
} // namespace mrover

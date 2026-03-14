#include "serial/fdcan.hpp"
#include "serial/smbus.hpp"

extern FDCAN_HandleTypeDef hfdcan1;
extern I2C_HandleTypeDef hi2c3;

namespace mrover {

    FDCAN m_fdcan;
    SMBus m_i2c;

    void fdcan_test() {
    }

    void i2c_test() {
    }

    void main() {
        m_fdcan = FDCAN(&hfdcan1);
        m_i2c = SMBus(&hi2c3);

        fdcan_test();

        i2c_test();

        while (true) {
        }
    }
} // namespace mrover

extern "C" {
void HAL_PostInit() {
    mrover::main();
}
}

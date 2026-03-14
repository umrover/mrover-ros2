#include "auton_led.hpp"
#include "main.h"

namespace mrover {
    AutonLed auton_led{
            Pin(AUTON_LED_R_GPIO_Port, AUTON_LED_R_Pin),
            Pin(AUTON_LED_G_GPIO_Port, AUTON_LED_G_Pin),
            Pin(AUTON_LED_B_GPIO_Port, AUTON_LED_B_Pin),
    };

    void init() {
    }

    void loop() {
    }

} // namespace mrover

void PostInit() {
    mrover::init();
}

void Loop() {
    mrover::loop();
}

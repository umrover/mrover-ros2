#include <pdlb.hpp>
#include <cstdint>
#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;

extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim1;

namespace mrover {

    // NOTE: Change this for the PDLB controller
    constexpr static std::uint8_t DEVICE_ID = 0x50;

    // Usually this is the Jetson
    constexpr static std::uint8_t DESTINATION_DEVICE_ID = 0x10;

    FDCAN<InBoundPDLBMessage> fdcan_bus;
    PDLB pdlb;

    void init() {
        AutonLed auton_led = AutonLed{
        	Pin{AUTON_LED_R_GPIO_Port, AUTON_LED_R_Pin},
			Pin{AUTON_LED_G_GPIO_Port, AUTON_LED_G_Pin},
			Pin{AUTON_LED_B_GPIO_Port, AUTON_LED_B_Pin}
        };
        auton_led.change_state(true, false, true, false);
        fdcan_bus = FDCAN<InBoundPDLBMessage>{&hfdcan1};
        fdcan_bus.start();
        pdlb = PDLB{fdcan_bus, auton_led,  Pin{ARM_LASER_GPIO_Port, ARM_LASER_Pin}};
    }

    void blink_led_if_applicable() {
		pdlb.blink_led_if_applicable();
	}

    void receive_message() {
    	if (std::optional received = fdcan_bus.receive()) {
			auto const& [header, message] = received.value();
			auto messageId = std::bit_cast<FDCAN<InBoundPDLBMessage>::MessageId>(header.Identifier);
			if (messageId.destination == DEVICE_ID)
				pdlb.receive(message);
		}
	}


} // namespace mrover

void init() {
    mrover::init();
}

void blink_led_if_applicable() {
	mrover::blink_led_if_applicable();
}

void receive_message() {
	mrover::receive_message();
//	mrover::pdlb.blink_led_if_applicable();
}


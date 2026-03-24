#pragma once

#include "CO2Sensor.hpp"
#include "OxygenSensor.hpp"
#include "OzoneSensor.hpp"
#include "UVSensor.hpp"
#include "THPSensor.hpp"
#include <CANBus1.hpp>
#include <hw/pin.hpp>
#include <config.hpp>
#include <queue>
#include <logger.hpp>

namespace mrover {
    enum Sensor {
        sensor_co2_rx = 0,
        sensor_co2_tx = 1,
        sensor_thp = 2,
        sensor_ozone = 3,
        sensor_oxygen = 4,
        sensor_uv = 5,
    };

    #pragma pack(push,1)
    struct SensorStates {
        uint8_t uv_state : 1;
        uint8_t thp_state : 1;
        uint8_t oxygen_state : 1;
        uint8_t ozone_state : 1;
        uint8_t co2_state : 1;
    };
    #pragma pack(pop)

    class ScienceBoard {
    private:
        THP thp_sensor{};
        CO2Sensor co2_sensor{};
        OzoneSensor ozone_sensor{};
        OxygenSensor oxygen_sensor{};
        UVSensor uv_sensor{};
        Pin can_tx{};
        Pin can_rx{};
        Pin dbg_led1{};
        Pin dbg_led2{};
        Pin dbg_led3{};
        CANBus1Handler can_handler{};
        sb_config_t config;
        SensorStates sensor_states{1,1,1,1,1};

         // initialize i2c sensors
        void init() {
            if (!ozone_sensor.init())
                flag_sensor(sensor_ozone);
            if (!oxygen_sensor.init())
                flag_sensor(sensor_oxygen);
            if (!thp_sensor.init())
                flag_sensor(sensor_thp);
            if (!co2_sensor.init())
                flag_sensor(sensor_co2_tx);
        }

        // clears all sensor faults by resetting all bits to 1
        void clear_faults(std::queue<Sensor>& i2c_queue) {
            if (!sensor_states.co2_state)
                i2c_queue.push(Sensor::sensor_co2_rx);
            if (!sensor_states.thp_state)
                i2c_queue.push(Sensor::sensor_thp);
            if (!sensor_states.oxygen_state)
                i2c_queue.push(Sensor::sensor_oxygen);
            if (!sensor_states.ozone_state)
                i2c_queue.push(Sensor::sensor_ozone);

            sensor_states = {1,1,1,1,1};
        }

        // deinitialize peripherals and reset mcu
        static void reset() {
            HAL_DeInit();
            NVIC_SystemReset();
        }

        // need empty function for any unrelated messages
        template<typename T>
        void handle(T const& _, std::queue<Sensor>& i2c_queue) {
        }

        // handles a reset command
        void handle(const SCIResetCommand& cmd, std::queue<Sensor>& i2c_queue) {
            if (cmd.clear_faults)
                clear_faults(i2c_queue);
            else if (cmd.reset)
                reset();
        }

    public:
        ScienceBoard() = default;

        ScienceBoard(
                THP& thp_in, 
                CO2Sensor& co2_in, 
                OzoneSensor& ozone_in,
                OxygenSensor& oxygen_in,
                UVSensor& uv_in,
                Pin& can_tx_in,
                Pin& can_rx_in,
                Pin& dbg_led1_in,
                Pin& dbg_led2_in,
                Pin& dbg_led3_in,
                CANBus1Handler& can_handler_in) : thp_sensor(thp_in),
                                                    co2_sensor(co2_in),
                                                    ozone_sensor(ozone_in), 
                                                    oxygen_sensor(oxygen_in),
                                                    uv_sensor(uv_in),
                                                    can_tx(can_tx_in),
                                                    can_rx(can_rx_in),
                                                    dbg_led1(dbg_led1_in),
                                                    dbg_led2(dbg_led2_in),
                                                    dbg_led3(dbg_led3_in),
                                                    can_handler(can_handler_in) {
            init();
        }

        // tries to reinitialize any sensors with an error state
        void reinit_sensors(std::queue<Sensor>& i2c_queue) {
            auto& logger = Logger::instance();
            if (!sensor_states.co2_state) {
                logger.info("Attempting CO2 reinit...");
                if (co2_sensor.init()) {
                    i2c_queue.push(sensor_co2_tx);
                    sensor_states.co2_state = 1;
                    logger.info("CO2 reinit successful");
                } else {
                    logger.info("CO2 reinit failed");
                }
            }
            if (!sensor_states.thp_state) {
                logger.info("Attempting THP reinit...");
                if (thp_sensor.init()) {
                    sensor_states.thp_state = 1;
                    i2c_queue.push(sensor_thp);
                    logger.info("THP reinit successful");
                } else {
                    logger.info("THP reinit failed");
                }
            }
            if (!sensor_states.oxygen_state) {
                logger.info("Attempting oxygen reinit...");
                if (oxygen_sensor.init()) {
                    sensor_states.oxygen_state = 1;
                    i2c_queue.push(sensor_oxygen);
                    logger.info("Oxygen reinit successful");
                } else {
                    logger.info("Oxygen reinit failed");
                }
            }
            if (!sensor_states.ozone_state) {
                logger.info("Attempting ozone reinit...");
                if (ozone_sensor.init()) {
                    sensor_states.ozone_state = 1;
                    i2c_queue.push(sensor_ozone);
                    logger.info("Ozone reinit successful");
                } else {
                    logger.info("Ozone reinit failed");
                }
            }
        }

        bool check_sensor(Sensor sensor) const {
            if (sensor == sensor_co2_tx || sensor == sensor_co2_rx)
                return sensor_states.co2_state;
            else if (sensor == sensor_thp)
                return sensor_states.thp_state;
            else if (sensor == sensor_oxygen)
                return sensor_states.oxygen_state;
            else if (sensor == sensor_ozone)
                return sensor_states.ozone_state;
            else if (sensor == sensor_uv)
                return sensor_states.uv_state;
            else
                return false;
        }

        void update_sensor (Sensor sensor) {
            if (sensor == sensor_co2_rx)
                co2_sensor.update_co2();
            else if (sensor == sensor_thp)
                thp_sensor.update_thp();
            else if (sensor == sensor_oxygen)
                oxygen_sensor.update_oxygen();
            else if (sensor == sensor_ozone)
                ozone_sensor.update_ozone();
            else if (sensor == sensor_uv)
                uv_sensor.update_uv();
        }

        void poll_sensor (Sensor sensor) {
            if (sensor == sensor_co2_tx)
                co2_sensor.request_co2();
            else if (sensor == sensor_co2_rx)
                co2_sensor.receive_buf();
            else if (sensor == sensor_thp)
                thp_sensor.read_thp();
            else if (sensor == sensor_oxygen)
                oxygen_sensor.read_oxygen();
            else if (sensor == sensor_ozone)
                ozone_sensor.read_ozone();
            else if (sensor == sensor_uv)
                uv_sensor.sample_sensor();
        }

        void flag_sensor (Sensor sensor) {
            if (sensor == sensor_co2_tx || sensor == sensor_co2_rx)
                sensor_states.co2_state = 0;
            else if (sensor == sensor_thp)
                sensor_states.thp_state = 0;
            else if (sensor == sensor_oxygen)
                sensor_states.oxygen_state = 0;
            else if (sensor == sensor_ozone)
                sensor_states.ozone_state = 0;
            else if (sensor == sensor_uv)
                sensor_states.uv_state = 0;
        }

        void send_sensor_data() {
            can_tx.set();
            const CANBus1Msg_t msg = SCISensorData(
                                                uv_sensor.get_current_uv(), 
                                                thp_sensor.get_thp().temp, 
                                                thp_sensor.get_thp().humidity, 
                                                thp_sensor.get_thp().pressure, 
                                                oxygen_sensor.get_oxygen(), 
                                                ozone_sensor.get_ozone(),
                                                co2_sensor.get_co2());
            // can_handler.send(msg, config.get<sb_config_t::can_id>(), config.get<sb_config_t::host_can_id>());
            can_handler.send(msg, 0x40, 0x10);
            can_tx.reset();
        }

        void send_sensor_state() {
            can_tx.set();
            const CANBus1Msg_t msg = SCISensorState(sensor_states.uv_state, 
                                                    sensor_states.thp_state,
                                                    sensor_states.oxygen_state, 
                                                    sensor_states.ozone_state, 
                                                    sensor_states.co2_state);
            // can_handler.send(msg, config.get<sb_config_t::can_id>(), config.get<sb_config_t::host_can_id>());
            can_handler.send(msg, 0x40, 0x10);
            can_tx.reset();
        }

        void handle_request(std::queue<Sensor>& i2c_queue) {
            auto const recv = can_handler.receive();
            if (recv) {
                can_rx.set();
                std::visit([this, &i2c_queue](auto&& value) { handle(value, i2c_queue); }, *recv);
                can_rx.reset();
            }
        }
    };
}
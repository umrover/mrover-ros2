#pragma once

#include <cstdint>
#include <span>
#include <string_view>

#ifdef STM32
#include "main.h"
#endif // STM32

namespace mrover {

#ifdef HAL_I2C_MODULE_ENABLED
    class SMBus {
        constexpr static std::size_t I2C_MAX_FRAME_SIZE = 32;
        constexpr static uint32_t I2C_TIMEOUT = 500, I2C_REBOOT_DELAY = 5;

    public:
        SMBus() = default;

        explicit SMBus(I2C_HandleTypeDef* hi2c) : m_i2c{hi2c} {}

        void reboot() const {
            HAL_I2C_DeInit(m_i2c);
            HAL_Delay(I2C_REBOOT_DELAY);
            HAL_I2C_Init(m_i2c);
        }

        void set_timeout(uint32_t timeout) { m_timeout = timeout; }
        [[nodiscard]] auto get_timeout() const -> uint32_t { return m_timeout; }

        auto blocking_transmit(uint16_t address, std::string_view data) -> bool {
            if (HAL_I2C_Master_Transmit(m_i2c, address << 1, const_cast<uint8_t*>(reinterpret_cast<uint8_t const*>(data.data())),
                                        static_cast<uint16_t>(data.size()), m_timeout) != HAL_OK) {
                return false;
            }

            return true;
        }

        auto blocking_receive(uint16_t address, std::span<uint8_t> data) -> bool {
            if (HAL_I2C_Master_Receive(m_i2c, address << 1 | 1, data.data(), data.size(), m_timeout) != HAL_OK) {
                return false;
            }
            return true;
        }

        auto blocking_transact(uint16_t address, std::string_view send_data, std::span<uint8_t> recv_data) -> bool {
            bool status = blocking_transmit(address, send_data);

            if (!status) {
                return false;
            }

            // reads from address sent above
            status = blocking_receive(address, recv_data);
            if (!status) {
                return false;
            }

            return true;
        }

        auto wait_until_ready_or_timeout() -> bool {
            if (HAL_I2C_GetState(m_i2c) != HAL_I2C_STATE_READY) {
                uint32_t tickstart = HAL_GetTick();
                while (HAL_I2C_GetState(m_i2c) != HAL_I2C_STATE_READY) {
                    if (m_timeout != HAL_MAX_DELAY) {
                        if (((HAL_GetTick() - tickstart) > m_timeout) || (m_timeout == 0U)) {
                            // TODO: (owen) timeout behavior
                            return false;
                        }
                    }
                }
            }
            return true;
        }

        auto async_transmit(uint16_t const address, std::string_view data) -> bool {
            if (!wait_until_ready_or_timeout()) {
                return false;
            }

            if (HAL_I2C_Master_Transmit_DMA(m_i2c, address << 1, const_cast<uint8_t*>(reinterpret_cast<uint8_t const*>(data.data())), data.size()) != HAL_OK) {
                return false;
            }
            return true;
        }

        auto async_receive(uint16_t const address, std::span<uint8_t> data) -> bool {
            if (!wait_until_ready_or_timeout()) {
                return false;
            }

            if (HAL_I2C_Master_Receive_DMA(m_i2c, address << 1 | 1, data.data(), data.size()) != HAL_OK) {
                return false;
            }

            return true;
        }

    private:
        I2C_HandleTypeDef* m_i2c{};
        uint32_t m_timeout = I2C_TIMEOUT;
    };
#else  // HAL_I2C_MODULE_ENABLED
    class __attribute__((unavailable("enable 'I2C' in STM32CubeMX to use mrover::SMBus"))) SMBus {
    public:
        template<typename... Args>
        explicit SMBus(Args&&... args) {}
    };
#endif // HAL_I2C_MODULE_ENABLED

} // namespace mrover

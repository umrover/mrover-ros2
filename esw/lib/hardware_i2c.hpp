#pragma once

#include <cstdint>
#include <optional>

#include "hardware.hpp"


namespace mrover {

    constexpr static std::size_t I2C_MAX_FRAME_SIZE = 32;

    template<typename T>
    concept IsI2CSerializable = IsSerializable<T> &&
                                sizeof(T) <= I2C_MAX_FRAME_SIZE;

    template<IsI2CSerializable TSend, IsI2CSerializable TReceive>
    class SMBus {
        constexpr static std::uint32_t I2C_TIMEOUT = 500, I2C_REBOOT_DELAY = 5;

    public:
        SMBus() = default;

        explicit SMBus(I2C_HandleTypeDef* hi2c) : m_i2c{hi2c} {}

        void reboot() const {
            HAL_I2C_DeInit(m_i2c);
            HAL_Delay(I2C_REBOOT_DELAY);
            HAL_I2C_Init(m_i2c);
        }

        template<IsI2CSerializable ImmediateTSend = TSend>
        auto blocking_transmit(std::uint16_t address, ImmediateTSend send) -> void {
            HAL_I2C_Master_Transmit(m_i2c, address << 1, send,
                                    sizeof(send), I2C_TIMEOUT);
        }

        auto blocking_receive(std::uint16_t address) -> std::optional<TReceive> {
            // TODO(quintin): Error handling? Shouldn't this return an optional
            if (TReceive receive{}; HAL_I2C_Master_Receive(m_i2c, address << 1 | 1, address_of<std::uint8_t>(receive), sizeof(receive), I2C_TIMEOUT) != HAL_OK) {
                return std::nullopt;
            } else {
                return receive;
            }
        }

        auto blocking_transact(std::uint16_t address, TSend const& send) -> std::optional<TReceive> {
            if (HAL_I2C_Master_Transmit(m_i2c, address << 1, address_of<std::uint8_t>(send), sizeof(send), I2C_TIMEOUT) != HAL_OK) {
                // reboot();
                return std::nullopt;
            }

            // reads from address sent above
            if (TReceive receive{};
                HAL_I2C_Master_Receive(m_i2c, address << 1 | 0b1,
                                       address_of<TReceive>(receive), sizeof(receive),
                                       I2C_TIMEOUT) != HAL_OK) {
                reboot();
                return std::nullopt;
            } else {
                return receive;
            }
        }

        auto async_transmit(std::uint16_t const address, TSend send) -> void {
        	if (HAL_I2C_GetState(m_i2c) != HAL_I2C_STATE_READY){
				uint32_t tickstart = HAL_GetTick();
				while (HAL_I2C_GetState(m_i2c) != HAL_I2C_STATE_READY) {
					if (I2C_TIMEOUT != HAL_MAX_DELAY) {
					  if (((HAL_GetTick() - tickstart) > I2C_TIMEOUT) || (I2C_TIMEOUT == 0U)) {
						  // TODO: (owen) timeout behavior
						  return;
					  }
					}
				}
			}

			if (HAL_I2C_Master_Transmit_DMA(m_i2c, address << 1, address_of<uint8_t>(send), sizeof(send)) != HAL_OK) {
				return;
			}
        }

        auto async_receive(std::uint16_t const address) -> void {
        	if (HAL_I2C_GetState(m_i2c) != HAL_I2C_STATE_READY){
				uint32_t tickstart = HAL_GetTick();
				while (HAL_I2C_GetState(m_i2c) != HAL_I2C_STATE_READY) {
					if (I2C_TIMEOUT != HAL_MAX_DELAY) {
					  if (((HAL_GetTick() - tickstart) > I2C_TIMEOUT) || (I2C_TIMEOUT == 0U)) {
						  // TODO: (owen) timeout behavior
						  return;
					  }
					}
				}
			}
            m_receive_buffer = TReceive{};

            if (HAL_I2C_Master_Receive_DMA(m_i2c, address << 1 | 1, address_of<uint8_t>(*m_receive_buffer), sizeof(*m_receive_buffer)) != HAL_OK) {
            	m_receive_buffer = std::nullopt;
            	return;
            }
        }

        auto get_buffer() const -> std::optional<TReceive> {
            return m_receive_buffer;
        }


    private:
        I2C_HandleTypeDef* m_i2c{};
        std::optional<TReceive> m_receive_buffer{};
    };

} // namespace mrover

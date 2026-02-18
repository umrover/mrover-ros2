#pragma once

#include <algorithm>
#include <cstdint>
#include <span>
#include <string_view>

#include <util.hpp>

#ifdef STM32
#include "main.h"
#endif // STM32

namespace mrover {

    constexpr static std::size_t FDCAN_MAX_FRAME_SIZE = 64;

#ifdef HAL_FDCAN_MODULE_ENABLED
    class FDCAN {
    public:
        enum class FilterIdType {
            Standard,
            Extended,
        };

        enum class FilterAction {
            Accept,
            Reject,
        };

        enum class FilterMode {
            Range,
            Dual,
            Mask,
        };

        struct Filter {
            uint32_t id1;
            uint32_t id2;

            FilterIdType id_type;
            FilterAction action;
            FilterMode mode;
        };

        struct FilterConfig {
            Filter const* begin = nullptr;
            Filter const* end = nullptr;

            FilterAction global_non_matching_std_action = FilterAction::Accept;
            FilterAction global_non_matching_ext_action = FilterAction::Accept;
            FilterAction global_remote_std_action = FilterAction::Accept;
            FilterAction global_remote_ext_action = FilterAction::Accept;
        };

        struct Options {
            // TODO: add more options to make this driver more configurable
            FilterConfig filter_config{};

            bool delay_compensation = false; // NOTE: needs to be on to enable BRS
            uint32_t tdc_offset = 0;         // 13 with moteus
            uint32_t tdc_filter = 0;         // 1 with moteus

            Options() {};
        };

        FDCAN() = default;

        explicit FDCAN(FDCAN_HandleTypeDef* fdcan, Options const& options = Options())
            : m_fdcan{fdcan}, m_options{options} {
            init();
        }

        auto init() -> void {
            if (m_options.delay_compensation) {
                check(HAL_FDCAN_ConfigTxDelayCompensation(m_fdcan, m_options.tdc_offset, m_options.tdc_filter) == HAL_OK, Error_Handler);
                check(HAL_FDCAN_EnableTxDelayCompensation(m_fdcan) == HAL_OK, Error_Handler);
            } else {
                check(HAL_FDCAN_DisableTxDelayCompensation(m_fdcan) == HAL_OK, Error_Handler);
            }

            int std_filter_index = 0;
            int ext_filter_index = 0;

            FilterConfig const& filters = m_options.filter_config;

            std::for_each(filters.begin, filters.end,
                          [&](auto const& filter) {
                              FDCAN_FilterTypeDef f;

                              f.IdType = [&]() -> uint32_t {
                                  switch (filter.id_type) {
                                      case FilterIdType::Standard:
                                          return FDCAN_STANDARD_ID;
                                      case FilterIdType::Extended:
                                          return FDCAN_EXTENDED_ID;
                                  }
                                  Error_Handler();
                                  return FDCAN_STANDARD_ID;
                              }();

                              f.FilterIndex = [&]() -> uint32_t {
                                  switch (filter.id_type) {
                                      case FilterIdType::Standard: {
                                          return std_filter_index++;
                                      }
                                      case FilterIdType::Extended: {
                                          return ext_filter_index++;
                                      }
                                  }
                                  Error_Handler();
                                  return 0;
                              }();

                              f.FilterType = [&]() -> uint32_t {
                                  switch (filter.mode) {
                                      case FilterMode::Range:
                                          return FDCAN_FILTER_RANGE;
                                      case FilterMode::Dual:
                                          return FDCAN_FILTER_DUAL;
                                      case FilterMode::Mask:
                                          return FDCAN_FILTER_MASK;
                                  }
                                  Error_Handler();
                                  return FDCAN_FILTER_RANGE;
                              }();

                              f.FilterConfig = [&]() -> uint32_t {
                                  switch (filter.action) {
                                      case FilterAction::Accept:
                                          return FDCAN_FILTER_TO_RXFIFO0;
                                      case FilterAction::Reject:
                                          return FDCAN_FILTER_REJECT;
                                  }
                                  Error_Handler();
                                  return FDCAN_FILTER_REJECT;
                              }();

                              f.FilterID1 = filter.id1;
                              f.FilterID2 = filter.id2;

                              check(HAL_FDCAN_ConfigFilter(m_fdcan, &f) == HAL_OK, Error_Handler);
                          });

            auto map_filter_action = [](auto value) -> uint32_t {
                switch (value) {
                    case FilterAction::Accept: {
                        return FDCAN_ACCEPT_IN_RX_FIFO0;
                    }
                    case FilterAction::Reject: {
                        return FDCAN_REJECT;
                    }
                }
                Error_Handler();
                return FDCAN_REJECT;
            };

            auto map_remote_action = [](auto value) -> uint32_t {
                switch (value) {
                    case FilterAction::Accept: {
                        return FDCAN_FILTER_REMOTE;
                    }
                    case FilterAction::Reject: {
                        return FDCAN_REJECT_REMOTE;
                    }
                }
                Error_Handler();
                return FDCAN_REJECT_REMOTE;
            };

            check(HAL_FDCAN_ConfigGlobalFilter(m_fdcan,
                                               map_filter_action(filters.global_non_matching_std_action),
                                               map_filter_action(filters.global_non_matching_ext_action),
                                               map_remote_action(filters.global_remote_std_action),
                                               map_remote_action(filters.global_remote_ext_action)) == HAL_OK,
                  Error_Handler);

            check(HAL_FDCAN_ActivateNotification(m_fdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) == HAL_OK, Error_Handler);
            check(HAL_FDCAN_Start(m_fdcan) == HAL_OK, Error_Handler);
        }

        /**
         * \brief   Attempt to pop a message from the receive queue
         * \return  True if message received from queue, false otherwise
         */
        [[nodiscard]] auto receive(FDCAN_RxHeaderTypeDef* header, std::span<uint8_t> data) const -> bool {
            if (HAL_FDCAN_GetRxFifoFillLevel(m_fdcan, FDCAN_RX_FIFO0) == 0)
                return false;

            if (HAL_FDCAN_GetRxMessage(m_fdcan, FDCAN_RX_FIFO0, header, data.data()) != HAL_OK) {
                return false;
            }
            return true;
        }

        auto messages_to_process() const -> uint32_t {
            return HAL_FDCAN_GetRxFifoFillLevel(m_fdcan, FDCAN_RX_FIFO0);
        }

        /**
         * \brief Needed since only certain frame sizes less than or equal to 64 bytes are allowed
         */
        [[nodiscard]] constexpr static auto nearest_fitting_can_fd_frame_size(std::size_t const size) -> uint32_t {
            if (size <= 0) return FDCAN_DLC_BYTES_0;
            if (size <= 1) return FDCAN_DLC_BYTES_1;
            if (size <= 2) return FDCAN_DLC_BYTES_2;
            if (size <= 3) return FDCAN_DLC_BYTES_3;
            if (size <= 4) return FDCAN_DLC_BYTES_4;
            if (size <= 5) return FDCAN_DLC_BYTES_5;
            if (size <= 6) return FDCAN_DLC_BYTES_6;
            if (size <= 7) return FDCAN_DLC_BYTES_7;
            if (size <= 8) return FDCAN_DLC_BYTES_8;
            if (size <= 12) return FDCAN_DLC_BYTES_12;
            if (size <= 16) return FDCAN_DLC_BYTES_16;
            if (size <= 20) return FDCAN_DLC_BYTES_20;
            if (size <= 24) return FDCAN_DLC_BYTES_24;
            if (size <= 32) return FDCAN_DLC_BYTES_32;
            if (size <= 48) return FDCAN_DLC_BYTES_48;
            if (size <= 64) return FDCAN_DLC_BYTES_64;
            return 0;
        }

        void send(uint32_t const id, std::string_view const data) {
            if (m_last_tx_request) {
                HAL_FDCAN_AbortTxRequest(m_fdcan, m_last_tx_request);
            }

            FDCAN_TxHeaderTypeDef const header{
                    .Identifier = id,
                    .IdType = FDCAN_EXTENDED_ID,
                    .TxFrameType = FDCAN_DATA_FRAME,
                    .DataLength = nearest_fitting_can_fd_frame_size(data.size()),
                    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
                    .BitRateSwitch = FDCAN_BRS_ON,
                    .FDFormat = FDCAN_FD_CAN,
                    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
            };

            if (HAL_FDCAN_AddMessageToTxFifoQ(m_fdcan, &header, const_cast<uint8_t*>(reinterpret_cast<uint8_t const*>(data.data()))) != HAL_OK) {
                Error_Handler();
            }

            m_last_tx_request = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(m_fdcan);
        }

        auto reset() const -> void {
            HAL_FDCAN_Stop(m_fdcan);
            HAL_FDCAN_Start(m_fdcan);
        }

    private:
        FDCAN_HandleTypeDef* m_fdcan{};
        Options m_options{};
        uint32_t m_last_tx_request = 0;
    };
#else  // HAL_FDCAN_MODULE_ENABLED
    class __attribute__((unavailable("enable 'FDCAN' in STM32CubeMX to use mrover::FDCAN"))) FDCAN {
    public:
        template<typename... Args>
        explicit FDCAN(Args&&... args) {}
    };
#endif // HAL_FDCAN_MODULE_ENABLED

} // namespace mrover

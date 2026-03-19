/* science_test.hpp */
/* THIS FILE IS AUTO-GENERATED */
/* MODIFICATIONS WILL BE OVERWRITTEN ON BUILD */
/* Generated on: 2026-03-19 12:23:11 */

#pragma once


#include <cstdlib>
#include <cstdint>
#include <bit>
#include <cstring>
#include <variant>
#include <optional>
#ifdef STM32
#include <serial/fdcan.hpp>
#endif


namespace mrover {
    static constexpr uint32_t MOTEUS_PREFIX = 0x0000;
    static constexpr uint32_t MOTEUS_REPLY_MASK = 0x8000;
    static constexpr uint32_t CAN_NODE_MASK = 0xFFFF;
    static constexpr uint32_t CAN_DEST_ID_MASK = 0x00FF;
    static constexpr uint32_t CAN_SRC_ID_MASK = 0xFF00;
    static constexpr uint32_t CAN_DEST_ID_OFFSET = 0;
    static constexpr uint32_t CAN_SRC_ID_OFFSET = 8;

    
    class Science_Sensors {
    public:
        static constexpr uint32_t CAN_ID = 0x50;
        static constexpr uint32_t BASE_ID = CAN_ID & ~CAN_NODE_MASK;
        float Sensors_Temperature;
        float Sensors_Humidity;
        float Sensors_UV;
        float Sensors_Oxygen;

        uint8_t msg_arr[16];

        explicit Science_Sensors(uint8_t const * byte_arr) {
            std::memset(msg_arr, 0, 16);
            std::memcpy(msg_arr, byte_arr, 16);

            [[maybe_unused]] uint32_t i = 0;
            uint32_t temp_Sensors_Temperature = 0;

            for (uint8_t j = 0; j < 4; ++j) {
                temp_Sensors_Temperature |= static_cast<uint32_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            Sensors_Temperature = std::bit_cast<float>(temp_Sensors_Temperature);
            uint32_t temp_Sensors_Humidity = 0;

            for (uint8_t j = 0; j < 4; ++j) {
                temp_Sensors_Humidity |= static_cast<uint32_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            Sensors_Humidity = std::bit_cast<float>(temp_Sensors_Humidity);
            uint32_t temp_Sensors_UV = 0;

            for (uint8_t j = 0; j < 4; ++j) {
                temp_Sensors_UV |= static_cast<uint32_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            Sensors_UV = std::bit_cast<float>(temp_Sensors_UV);
            uint32_t temp_Sensors_Oxygen = 0;

            for (uint8_t j = 0; j < 4; ++j) {
                temp_Sensors_Oxygen |= static_cast<uint32_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            Sensors_Oxygen = std::bit_cast<float>(temp_Sensors_Oxygen);
        }

        Science_Sensors(
            float const Sensors_Temperature_in, 
            float const Sensors_Humidity_in, 
            float const Sensors_UV_in, 
            float const Sensors_Oxygen_in
        ) :
            Sensors_Temperature(Sensors_Temperature_in), 
            Sensors_Humidity(Sensors_Humidity_in), 
            Sensors_UV(Sensors_UV_in), 
            Sensors_Oxygen(Sensors_Oxygen_in)
         {
            std::memset(msg_arr, 0, 16);

            [[maybe_unused]] uint32_t i = 0;
            for (uint8_t j = 0; j < 4; ++j) {
                msg_arr[i] = (std::bit_cast<uint32_t>(Sensors_Temperature) >> (8*j)) & 0xFF;
                ++i;
            }
            for (uint8_t j = 0; j < 4; ++j) {
                msg_arr[i] = (std::bit_cast<uint32_t>(Sensors_Humidity) >> (8*j)) & 0xFF;
                ++i;
            }
            for (uint8_t j = 0; j < 4; ++j) {
                msg_arr[i] = (std::bit_cast<uint32_t>(Sensors_UV) >> (8*j)) & 0xFF;
                ++i;
            }
            for (uint8_t j = 0; j < 4; ++j) {
                msg_arr[i] = (std::bit_cast<uint32_t>(Sensors_Oxygen) >> (8*j)) & 0xFF;
                ++i;
            }
        }
    };
    
    class Science_ISHOutbound {
    public:
        static constexpr uint32_t CAN_ID = 0x51;
        static constexpr uint32_t BASE_ID = CAN_ID & ~CAN_NODE_MASK;
        float ISH_Heater1Temp;
        float ISH_Heater2Temp;
        uint8_t ISH_Heater1State;
        uint8_t ISH_Heater2State;
        uint8_t ISH_WLED1State;
        uint8_t ISH_WLED2State;

        uint8_t msg_arr[12];

        explicit Science_ISHOutbound(uint8_t const * byte_arr) {
            std::memset(msg_arr, 0, 12);
            std::memcpy(msg_arr, byte_arr, 12);

            [[maybe_unused]] uint32_t i = 0;
            uint32_t temp_ISH_Heater1Temp = 0;

            for (uint8_t j = 0; j < 4; ++j) {
                temp_ISH_Heater1Temp |= static_cast<uint32_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            ISH_Heater1Temp = std::bit_cast<float>(temp_ISH_Heater1Temp);
            uint32_t temp_ISH_Heater2Temp = 0;

            for (uint8_t j = 0; j < 4; ++j) {
                temp_ISH_Heater2Temp |= static_cast<uint32_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            ISH_Heater2Temp = std::bit_cast<float>(temp_ISH_Heater2Temp);
            ISH_Heater1State = (byte_arr[i] >> 0) & 0b1;
            ISH_Heater2State = (byte_arr[i] >> 1) & 0b1;
            ISH_WLED1State = (byte_arr[i] >> 2) & 0b1;
            ISH_WLED2State = (byte_arr[i] >> 3) & 0b1;
        }

        Science_ISHOutbound(
            float const ISH_Heater1Temp_in, 
            float const ISH_Heater2Temp_in, 
            uint8_t const ISH_Heater1State_in, 
            uint8_t const ISH_Heater2State_in, 
            uint8_t const ISH_WLED1State_in, 
            uint8_t const ISH_WLED2State_in
        ) :
            ISH_Heater1Temp(ISH_Heater1Temp_in), 
            ISH_Heater2Temp(ISH_Heater2Temp_in), 
            ISH_Heater1State(ISH_Heater1State_in), 
            ISH_Heater2State(ISH_Heater2State_in), 
            ISH_WLED1State(ISH_WLED1State_in), 
            ISH_WLED2State(ISH_WLED2State_in)
         {
            std::memset(msg_arr, 0, 12);

            [[maybe_unused]] uint32_t i = 0;
            for (uint8_t j = 0; j < 4; ++j) {
                msg_arr[i] = (std::bit_cast<uint32_t>(ISH_Heater1Temp) >> (8*j)) & 0xFF;
                ++i;
            }
            for (uint8_t j = 0; j < 4; ++j) {
                msg_arr[i] = (std::bit_cast<uint32_t>(ISH_Heater2Temp) >> (8*j)) & 0xFF;
                ++i;
            }

            if(ISH_Heater1State) {
                msg_arr[i] |= (1 << 0);
            } else {
                msg_arr[i] &= ~(1 << 0);
            }

            if(ISH_Heater2State) {
                msg_arr[i] |= (1 << 1);
            } else {
                msg_arr[i] &= ~(1 << 1);
            }

            if(ISH_WLED1State) {
                msg_arr[i] |= (1 << 2);
            } else {
                msg_arr[i] &= ~(1 << 2);
            }

            if(ISH_WLED2State) {
                msg_arr[i] |= (1 << 3);
            } else {
                msg_arr[i] &= ~(1 << 3);
            }
        }
    };
    
    class Science_ISHInbound {
    public:
        static constexpr uint32_t CAN_ID = 0x52;
        static constexpr uint32_t BASE_ID = CAN_ID & ~CAN_NODE_MASK;
        uint8_t ISH_Heater1Enable;
        uint8_t ISH_Heater2Enable;
        uint8_t ISH_Heater1EnableAS;
        uint8_t ISH_Heater2EnableAS;
        uint8_t ISH_WLED1Enable;
        uint8_t ISH_WLED2Enable;

        uint8_t msg_arr[1];

        explicit Science_ISHInbound(uint8_t const * byte_arr) {
            std::memset(msg_arr, 0, 1);
            std::memcpy(msg_arr, byte_arr, 1);

            [[maybe_unused]] uint32_t i = 0;
            ISH_Heater1Enable = (byte_arr[i] >> 0) & 0b1;
            ISH_Heater2Enable = (byte_arr[i] >> 1) & 0b1;
            ISH_Heater1EnableAS = (byte_arr[i] >> 2) & 0b1;
            ISH_Heater2EnableAS = (byte_arr[i] >> 3) & 0b1;
            ISH_WLED1Enable = (byte_arr[i] >> 4) & 0b1;
            ISH_WLED2Enable = (byte_arr[i] >> 5) & 0b1;
        }

        Science_ISHInbound(
            uint8_t const ISH_Heater1Enable_in, 
            uint8_t const ISH_Heater2Enable_in, 
            uint8_t const ISH_Heater1EnableAS_in, 
            uint8_t const ISH_Heater2EnableAS_in, 
            uint8_t const ISH_WLED1Enable_in, 
            uint8_t const ISH_WLED2Enable_in
        ) :
            ISH_Heater1Enable(ISH_Heater1Enable_in), 
            ISH_Heater2Enable(ISH_Heater2Enable_in), 
            ISH_Heater1EnableAS(ISH_Heater1EnableAS_in), 
            ISH_Heater2EnableAS(ISH_Heater2EnableAS_in), 
            ISH_WLED1Enable(ISH_WLED1Enable_in), 
            ISH_WLED2Enable(ISH_WLED2Enable_in)
         {
            std::memset(msg_arr, 0, 1);

            [[maybe_unused]] uint32_t i = 0;

            if(ISH_Heater1Enable) {
                msg_arr[i] |= (1 << 0);
            } else {
                msg_arr[i] &= ~(1 << 0);
            }

            if(ISH_Heater2Enable) {
                msg_arr[i] |= (1 << 1);
            } else {
                msg_arr[i] &= ~(1 << 1);
            }

            if(ISH_Heater1EnableAS) {
                msg_arr[i] |= (1 << 2);
            } else {
                msg_arr[i] &= ~(1 << 2);
            }

            if(ISH_Heater2EnableAS) {
                msg_arr[i] |= (1 << 3);
            } else {
                msg_arr[i] &= ~(1 << 3);
            }

            if(ISH_WLED1Enable) {
                msg_arr[i] |= (1 << 4);
            } else {
                msg_arr[i] &= ~(1 << 4);
            }

            if(ISH_WLED2Enable) {
                msg_arr[i] |= (1 << 5);
            } else {
                msg_arr[i] &= ~(1 << 5);
            }
        }
    };
    

    template<typename T>
    concept is_can_message = requires(T const can_msg, uint8_t const* byte_arr) {
        { T{byte_arr} };
        { can_msg.msg_arr } -> std::convertible_to<const uint8_t*>;
        { T::BASE_ID } -> std::convertible_to<uint32_t const>;
    };

    using science_testMsg_t = std::variant<
        Science_ISHInbound,
        Science_ISHOutbound,
        Science_Sensors
    >;

#ifdef HAL_FDCAN_MODULE_ENABLED
    constexpr std::size_t dlc_to_size(uint32_t const dlc) {
        if (dlc <= FDCAN_DLC_BYTES_8) return dlc;
        if (dlc == FDCAN_DLC_BYTES_12) return 12;
        if (dlc == FDCAN_DLC_BYTES_16) return 16;
        if (dlc == FDCAN_DLC_BYTES_20) return 20;
        if (dlc == FDCAN_DLC_BYTES_24) return 24;
        if (dlc == FDCAN_DLC_BYTES_32) return 32;
        if (dlc == FDCAN_DLC_BYTES_48) return 48;
        if (dlc == FDCAN_DLC_BYTES_64) return 64;
        return 0;
    }

    class science_testHandler {
        FDCAN* m_fdcan;
    public:
        science_testHandler() = default;

        explicit science_testHandler(FDCAN* fdcan_driver)
            : m_fdcan{fdcan_driver} {}

        struct SendHandler {
            science_testHandler* m_can_handler;
            uint32_t const m_src_node_id;
            uint32_t const m_dest_node_id;

            SendHandler(science_testHandler* can_handler, uint32_t const src_node_id, uint32_t const dest_node_id)
                : m_can_handler{can_handler}, m_src_node_id{src_node_id}, m_dest_node_id{dest_node_id} {}

            template<typename can_msg_t>
            void operator()(can_msg_t const& message) const
                requires is_can_message<can_msg_t>
            {
                uint32_t const final_id = (can_msg_t::BASE_ID & ~CAN_NODE_MASK) | ((m_src_node_id << CAN_SRC_ID_OFFSET) & CAN_SRC_ID_MASK) | ((m_dest_node_id << CAN_DEST_ID_OFFSET) & CAN_DEST_ID_MASK);
                std::span<uint8_t const> const data(message.msg_arr, sizeof(message.msg_arr));
                this->m_can_handler->m_fdcan->send(final_id,
                    std::string_view(reinterpret_cast<char const*>(data.data()), data.size())
                );
            }
        };

        auto send(
            science_testMsg_t const& message_variant,
            uint32_t const src_node_id,
            uint32_t const dest_node_id
        ) -> void {
            SendHandler sender(this, src_node_id, dest_node_id);
            std::visit(sender, message_variant);
        }

        [[nodiscard]] auto receive() const -> std::optional<science_testMsg_t> {
            FDCAN_RxHeaderTypeDef header;
            uint8_t data_buffer[FDCAN_MAX_FRAME_SIZE];

            if (std::span<uint8_t> const data_span{data_buffer, FDCAN_MAX_FRAME_SIZE}; m_fdcan->receive(&header, data_span)) {
                uint32_t const received_base_id = header.Identifier & ~CAN_NODE_MASK;

                std::size_t const received_size = dlc_to_size(header.DataLength);
                std::span<uint8_t const> const received_data(data_buffer, received_size);

                switch (received_base_id) {
            
                    case (80 & ~CAN_NODE_MASK): {
                        return science_testMsg_t{ Science_Sensors{received_data.data()} };
                    }
                    case (81 & ~CAN_NODE_MASK): {
                        return science_testMsg_t{ Science_ISHOutbound{received_data.data()} };
                    }
                    case (82 & ~CAN_NODE_MASK): {
                        return science_testMsg_t{ Science_ISHInbound{received_data.data()} };
                    }
                    default:
                        return std::nullopt;
                }
            }
            return std::nullopt;
        }
    };
#else // HAL_FDCAN_MODULE_ENABLED
    // TODO: build jetson implementation here
    class __attribute__((unavailable("enable 'FDCAN' in STM32CubeMX to use mrover::science_testHandler"))) science_testHandler {
    public:
        template<typename... Args>
        explicit science_testHandler(Args&&...) {}
    };
#endif // HAL_FDCAN_MODULE_ENABLED

    template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
    template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;


} // namespace mrover
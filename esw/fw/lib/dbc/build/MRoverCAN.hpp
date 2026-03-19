/* MRoverCAN.hpp */
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

    
    class BMCModeCmd {
    public:
        static constexpr uint32_t CAN_ID = 0x100000;
        static constexpr uint32_t BASE_ID = CAN_ID & ~CAN_NODE_MASK;
        uint8_t mode;
        uint8_t enable;

        uint8_t msg_arr[2];

        explicit BMCModeCmd(uint8_t const * byte_arr) {
            std::memset(msg_arr, 0, 2);
            std::memcpy(msg_arr, byte_arr, 2);

            [[maybe_unused]] uint32_t i = 0;
            uint8_t temp_mode = 0;

            for (uint8_t j = 0; j < 1; ++j) {
                temp_mode |= static_cast<uint8_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            mode = std::bit_cast<uint8_t>(temp_mode);
            enable = (byte_arr[i] >> 0) & 0b1;
        }

        BMCModeCmd(
            uint8_t const mode_in, 
            uint8_t const enable_in
        ) :
            mode(mode_in), 
            enable(enable_in)
         {
            std::memset(msg_arr, 0, 2);

            [[maybe_unused]] uint32_t i = 0;
            for (uint8_t j = 0; j < 1; ++j) {
                msg_arr[i] = (std::bit_cast<uint8_t>(mode) >> (8*j)) & 0xFF;
                ++i;
            }

            if(enable) {
                msg_arr[i] |= (1 << 0);
            } else {
                msg_arr[i] &= ~(1 << 0);
            }
        }
    };
    
    class BMCTargetCmd {
    public:
        static constexpr uint32_t CAN_ID = 0x110000;
        static constexpr uint32_t BASE_ID = CAN_ID & ~CAN_NODE_MASK;
        float target;
        uint8_t target_valid;

        uint8_t msg_arr[5];

        explicit BMCTargetCmd(uint8_t const * byte_arr) {
            std::memset(msg_arr, 0, 5);
            std::memcpy(msg_arr, byte_arr, 5);

            [[maybe_unused]] uint32_t i = 0;
            uint32_t temp_target = 0;

            for (uint8_t j = 0; j < 4; ++j) {
                temp_target |= static_cast<uint32_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            target = std::bit_cast<float>(temp_target);
            target_valid = (byte_arr[i] >> 0) & 0b1;
        }

        BMCTargetCmd(
            float const target_in, 
            uint8_t const target_valid_in
        ) :
            target(target_in), 
            target_valid(target_valid_in)
         {
            std::memset(msg_arr, 0, 5);

            [[maybe_unused]] uint32_t i = 0;
            for (uint8_t j = 0; j < 4; ++j) {
                msg_arr[i] = (std::bit_cast<uint32_t>(target) >> (8*j)) & 0xFF;
                ++i;
            }

            if(target_valid) {
                msg_arr[i] |= (1 << 0);
            } else {
                msg_arr[i] &= ~(1 << 0);
            }
        }
    };
    
    class ESWConfigCmd {
    public:
        static constexpr uint32_t CAN_ID = 0xF00000;
        static constexpr uint32_t BASE_ID = CAN_ID & ~CAN_NODE_MASK;
        uint8_t address;
        uint32_t value;
        uint8_t apply;

        uint8_t msg_arr[6];

        explicit ESWConfigCmd(uint8_t const * byte_arr) {
            std::memset(msg_arr, 0, 6);
            std::memcpy(msg_arr, byte_arr, 6);

            [[maybe_unused]] uint32_t i = 0;
            uint8_t temp_address = 0;

            for (uint8_t j = 0; j < 1; ++j) {
                temp_address |= static_cast<uint8_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            address = std::bit_cast<uint8_t>(temp_address);
            uint32_t temp_value = 0;

            for (uint8_t j = 0; j < 4; ++j) {
                temp_value |= static_cast<uint32_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            value = std::bit_cast<uint32_t>(temp_value);
            apply = (byte_arr[i] >> 0) & 0b1;
        }

        ESWConfigCmd(
            uint8_t const address_in, 
            uint32_t const value_in, 
            uint8_t const apply_in
        ) :
            address(address_in), 
            value(value_in), 
            apply(apply_in)
         {
            std::memset(msg_arr, 0, 6);

            [[maybe_unused]] uint32_t i = 0;
            for (uint8_t j = 0; j < 1; ++j) {
                msg_arr[i] = (std::bit_cast<uint8_t>(address) >> (8*j)) & 0xFF;
                ++i;
            }
            for (uint8_t j = 0; j < 4; ++j) {
                msg_arr[i] = (std::bit_cast<uint32_t>(value) >> (8*j)) & 0xFF;
                ++i;
            }

            if(apply) {
                msg_arr[i] |= (1 << 0);
            } else {
                msg_arr[i] &= ~(1 << 0);
            }
        }
    };
    
    class BMCResetCmd {
    public:
        static constexpr uint32_t CAN_ID = 0x120000;
        static constexpr uint32_t BASE_ID = CAN_ID & ~CAN_NODE_MASK;
        uint8_t reset;
        uint8_t clear_faults;

        uint8_t msg_arr[1];

        explicit BMCResetCmd(uint8_t const * byte_arr) {
            std::memset(msg_arr, 0, 1);
            std::memcpy(msg_arr, byte_arr, 1);

            [[maybe_unused]] uint32_t i = 0;
            reset = (byte_arr[i] >> 0) & 0b1;
            clear_faults = (byte_arr[i] >> 1) & 0b1;
        }

        BMCResetCmd(
            uint8_t const reset_in, 
            uint8_t const clear_faults_in
        ) :
            reset(reset_in), 
            clear_faults(clear_faults_in)
         {
            std::memset(msg_arr, 0, 1);

            [[maybe_unused]] uint32_t i = 0;

            if(reset) {
                msg_arr[i] |= (1 << 0);
            } else {
                msg_arr[i] &= ~(1 << 0);
            }

            if(clear_faults) {
                msg_arr[i] |= (1 << 1);
            } else {
                msg_arr[i] &= ~(1 << 1);
            }
        }
    };
    
    class BMCMotorState {
    public:
        static constexpr uint32_t CAN_ID = 0x130000;
        static constexpr uint32_t BASE_ID = CAN_ID & ~CAN_NODE_MASK;
        uint8_t mode;
        uint8_t fault_code;
        float position;
        float velocity;
        float current;
        uint8_t limit_a;
        uint8_t limit_b;
        uint8_t is_stalled;

        uint8_t msg_arr[16];

        explicit BMCMotorState(uint8_t const * byte_arr) {
            std::memset(msg_arr, 0, 16);
            std::memcpy(msg_arr, byte_arr, 16);

            [[maybe_unused]] uint32_t i = 0;
            uint8_t temp_mode = 0;

            for (uint8_t j = 0; j < 1; ++j) {
                temp_mode |= static_cast<uint8_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            mode = std::bit_cast<uint8_t>(temp_mode);
            uint8_t temp_fault_code = 0;

            for (uint8_t j = 0; j < 1; ++j) {
                temp_fault_code |= static_cast<uint8_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            fault_code = std::bit_cast<uint8_t>(temp_fault_code);
            uint32_t temp_position = 0;

            for (uint8_t j = 0; j < 4; ++j) {
                temp_position |= static_cast<uint32_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            position = std::bit_cast<float>(temp_position);
            uint32_t temp_velocity = 0;

            for (uint8_t j = 0; j < 4; ++j) {
                temp_velocity |= static_cast<uint32_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            velocity = std::bit_cast<float>(temp_velocity);
            uint32_t temp_current = 0;

            for (uint8_t j = 0; j < 4; ++j) {
                temp_current |= static_cast<uint32_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            current = std::bit_cast<float>(temp_current);
            limit_a = (byte_arr[i] >> 0) & 0b1;
            limit_b = (byte_arr[i] >> 1) & 0b1;
            is_stalled = (byte_arr[i] >> 2) & 0b1;
        }

        BMCMotorState(
            uint8_t const mode_in, 
            uint8_t const fault_code_in, 
            float const position_in, 
            float const velocity_in, 
            float const current_in, 
            uint8_t const limit_a_in, 
            uint8_t const limit_b_in, 
            uint8_t const is_stalled_in
        ) :
            mode(mode_in), 
            fault_code(fault_code_in), 
            position(position_in), 
            velocity(velocity_in), 
            current(current_in), 
            limit_a(limit_a_in), 
            limit_b(limit_b_in), 
            is_stalled(is_stalled_in)
         {
            std::memset(msg_arr, 0, 16);

            [[maybe_unused]] uint32_t i = 0;
            for (uint8_t j = 0; j < 1; ++j) {
                msg_arr[i] = (std::bit_cast<uint8_t>(mode) >> (8*j)) & 0xFF;
                ++i;
            }
            for (uint8_t j = 0; j < 1; ++j) {
                msg_arr[i] = (std::bit_cast<uint8_t>(fault_code) >> (8*j)) & 0xFF;
                ++i;
            }
            for (uint8_t j = 0; j < 4; ++j) {
                msg_arr[i] = (std::bit_cast<uint32_t>(position) >> (8*j)) & 0xFF;
                ++i;
            }
            for (uint8_t j = 0; j < 4; ++j) {
                msg_arr[i] = (std::bit_cast<uint32_t>(velocity) >> (8*j)) & 0xFF;
                ++i;
            }
            for (uint8_t j = 0; j < 4; ++j) {
                msg_arr[i] = (std::bit_cast<uint32_t>(current) >> (8*j)) & 0xFF;
                ++i;
            }

            if(limit_a) {
                msg_arr[i] |= (1 << 0);
            } else {
                msg_arr[i] &= ~(1 << 0);
            }

            if(limit_b) {
                msg_arr[i] |= (1 << 1);
            } else {
                msg_arr[i] &= ~(1 << 1);
            }

            if(is_stalled) {
                msg_arr[i] |= (1 << 2);
            } else {
                msg_arr[i] &= ~(1 << 2);
            }
        }
    };
    
    class ESWProbe {
    public:
        static constexpr uint32_t CAN_ID = 0xF10000;
        static constexpr uint32_t BASE_ID = CAN_ID & ~CAN_NODE_MASK;
        uint32_t data;

        uint8_t msg_arr[4];

        explicit ESWProbe(uint8_t const * byte_arr) {
            std::memset(msg_arr, 0, 4);
            std::memcpy(msg_arr, byte_arr, 4);

            [[maybe_unused]] uint32_t i = 0;
            uint32_t temp_data = 0;

            for (uint8_t j = 0; j < 4; ++j) {
                temp_data |= static_cast<uint32_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            data = std::bit_cast<uint32_t>(temp_data);
        }

        ESWProbe(
            uint32_t const data_in
        ) :
            data(data_in)
         {
            std::memset(msg_arr, 0, 4);

            [[maybe_unused]] uint32_t i = 0;
            for (uint8_t j = 0; j < 4; ++j) {
                msg_arr[i] = (std::bit_cast<uint32_t>(data) >> (8*j)) & 0xFF;
                ++i;
            }
        }
    };
    
    class ESWAck {
    public:
        static constexpr uint32_t CAN_ID = 0xF20000;
        static constexpr uint32_t BASE_ID = CAN_ID & ~CAN_NODE_MASK;
        uint32_t data;

        uint8_t msg_arr[4];

        explicit ESWAck(uint8_t const * byte_arr) {
            std::memset(msg_arr, 0, 4);
            std::memcpy(msg_arr, byte_arr, 4);

            [[maybe_unused]] uint32_t i = 0;
            uint32_t temp_data = 0;

            for (uint8_t j = 0; j < 4; ++j) {
                temp_data |= static_cast<uint32_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            data = std::bit_cast<uint32_t>(temp_data);
        }

        ESWAck(
            uint32_t const data_in
        ) :
            data(data_in)
         {
            std::memset(msg_arr, 0, 4);

            [[maybe_unused]] uint32_t i = 0;
            for (uint8_t j = 0; j < 4; ++j) {
                msg_arr[i] = (std::bit_cast<uint32_t>(data) >> (8*j)) & 0xFF;
                ++i;
            }
        }
    };
    
    class SCISensorData {
    public:
        static constexpr uint32_t CAN_ID = 0x600000;
        static constexpr uint32_t BASE_ID = CAN_ID & ~CAN_NODE_MASK;
        float uv_index;
        float temperature;
        float humidity;
        float pressure;
        float oxygen;
        float ozone;
        float co2;

        uint8_t msg_arr[32];

        explicit SCISensorData(uint8_t const * byte_arr) {
            std::memset(msg_arr, 0, 32);
            std::memcpy(msg_arr, byte_arr, 32);

            [[maybe_unused]] uint32_t i = 0;
            uint32_t temp_uv_index = 0;

            for (uint8_t j = 0; j < 4; ++j) {
                temp_uv_index |= static_cast<uint32_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            uv_index = std::bit_cast<float>(temp_uv_index);
            uint32_t temp_temperature = 0;

            for (uint8_t j = 0; j < 4; ++j) {
                temp_temperature |= static_cast<uint32_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            temperature = std::bit_cast<float>(temp_temperature);
            uint32_t temp_humidity = 0;

            for (uint8_t j = 0; j < 4; ++j) {
                temp_humidity |= static_cast<uint32_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            humidity = std::bit_cast<float>(temp_humidity);
            uint32_t temp_pressure = 0;

            for (uint8_t j = 0; j < 4; ++j) {
                temp_pressure |= static_cast<uint32_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            pressure = std::bit_cast<float>(temp_pressure);
            uint32_t temp_oxygen = 0;

            for (uint8_t j = 0; j < 4; ++j) {
                temp_oxygen |= static_cast<uint32_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            oxygen = std::bit_cast<float>(temp_oxygen);
            uint32_t temp_ozone = 0;

            for (uint8_t j = 0; j < 4; ++j) {
                temp_ozone |= static_cast<uint32_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            ozone = std::bit_cast<float>(temp_ozone);
            uint32_t temp_co2 = 0;

            for (uint8_t j = 0; j < 4; ++j) {
                temp_co2 |= static_cast<uint32_t>(byte_arr[i]) << (8*j);
                ++i;
            }
            co2 = std::bit_cast<float>(temp_co2);
        }

        SCISensorData(
            float const uv_index_in, 
            float const temperature_in, 
            float const humidity_in, 
            float const pressure_in, 
            float const oxygen_in, 
            float const ozone_in, 
            float const co2_in
        ) :
            uv_index(uv_index_in), 
            temperature(temperature_in), 
            humidity(humidity_in), 
            pressure(pressure_in), 
            oxygen(oxygen_in), 
            ozone(ozone_in), 
            co2(co2_in)
         {
            std::memset(msg_arr, 0, 32);

            [[maybe_unused]] uint32_t i = 0;
            for (uint8_t j = 0; j < 4; ++j) {
                msg_arr[i] = (std::bit_cast<uint32_t>(uv_index) >> (8*j)) & 0xFF;
                ++i;
            }
            for (uint8_t j = 0; j < 4; ++j) {
                msg_arr[i] = (std::bit_cast<uint32_t>(temperature) >> (8*j)) & 0xFF;
                ++i;
            }
            for (uint8_t j = 0; j < 4; ++j) {
                msg_arr[i] = (std::bit_cast<uint32_t>(humidity) >> (8*j)) & 0xFF;
                ++i;
            }
            for (uint8_t j = 0; j < 4; ++j) {
                msg_arr[i] = (std::bit_cast<uint32_t>(pressure) >> (8*j)) & 0xFF;
                ++i;
            }
            for (uint8_t j = 0; j < 4; ++j) {
                msg_arr[i] = (std::bit_cast<uint32_t>(oxygen) >> (8*j)) & 0xFF;
                ++i;
            }
            for (uint8_t j = 0; j < 4; ++j) {
                msg_arr[i] = (std::bit_cast<uint32_t>(ozone) >> (8*j)) & 0xFF;
                ++i;
            }
            for (uint8_t j = 0; j < 4; ++j) {
                msg_arr[i] = (std::bit_cast<uint32_t>(co2) >> (8*j)) & 0xFF;
                ++i;
            }
        }
    };
    
    class SCISensorState {
    public:
        static constexpr uint32_t CAN_ID = 0x610000;
        static constexpr uint32_t BASE_ID = CAN_ID & ~CAN_NODE_MASK;
        uint8_t uv_state;
        uint8_t thp_state;
        uint8_t oxygen_state;
        uint8_t ozone_state;
        uint8_t co2_state;

        uint8_t msg_arr[1];

        explicit SCISensorState(uint8_t const * byte_arr) {
            std::memset(msg_arr, 0, 1);
            std::memcpy(msg_arr, byte_arr, 1);

            [[maybe_unused]] uint32_t i = 0;
            uv_state = (byte_arr[i] >> 0) & 0b1;
            thp_state = (byte_arr[i] >> 1) & 0b1;
            oxygen_state = (byte_arr[i] >> 2) & 0b1;
            ozone_state = (byte_arr[i] >> 3) & 0b1;
            co2_state = (byte_arr[i] >> 4) & 0b1;
        }

        SCISensorState(
            uint8_t const uv_state_in, 
            uint8_t const thp_state_in, 
            uint8_t const oxygen_state_in, 
            uint8_t const ozone_state_in, 
            uint8_t const co2_state_in
        ) :
            uv_state(uv_state_in), 
            thp_state(thp_state_in), 
            oxygen_state(oxygen_state_in), 
            ozone_state(ozone_state_in), 
            co2_state(co2_state_in)
         {
            std::memset(msg_arr, 0, 1);

            [[maybe_unused]] uint32_t i = 0;

            if(uv_state) {
                msg_arr[i] |= (1 << 0);
            } else {
                msg_arr[i] &= ~(1 << 0);
            }

            if(thp_state) {
                msg_arr[i] |= (1 << 1);
            } else {
                msg_arr[i] &= ~(1 << 1);
            }

            if(oxygen_state) {
                msg_arr[i] |= (1 << 2);
            } else {
                msg_arr[i] &= ~(1 << 2);
            }

            if(ozone_state) {
                msg_arr[i] |= (1 << 3);
            } else {
                msg_arr[i] &= ~(1 << 3);
            }

            if(co2_state) {
                msg_arr[i] |= (1 << 4);
            } else {
                msg_arr[i] &= ~(1 << 4);
            }
        }
    };
    
    class SCIResetCommand {
    public:
        static constexpr uint32_t CAN_ID = 0x510000;
        static constexpr uint32_t BASE_ID = CAN_ID & ~CAN_NODE_MASK;
        uint8_t reset;
        uint8_t clear_faults;

        uint8_t msg_arr[1];

        explicit SCIResetCommand(uint8_t const * byte_arr) {
            std::memset(msg_arr, 0, 1);
            std::memcpy(msg_arr, byte_arr, 1);

            [[maybe_unused]] uint32_t i = 0;
            reset = (byte_arr[i] >> 0) & 0b1;
            clear_faults = (byte_arr[i] >> 1) & 0b1;
        }

        SCIResetCommand(
            uint8_t const reset_in, 
            uint8_t const clear_faults_in
        ) :
            reset(reset_in), 
            clear_faults(clear_faults_in)
         {
            std::memset(msg_arr, 0, 1);

            [[maybe_unused]] uint32_t i = 0;

            if(reset) {
                msg_arr[i] |= (1 << 0);
            } else {
                msg_arr[i] &= ~(1 << 0);
            }

            if(clear_faults) {
                msg_arr[i] |= (1 << 1);
            } else {
                msg_arr[i] &= ~(1 << 1);
            }
        }
    };
    

    template<typename T>
    concept is_can_message = requires(T const can_msg, uint8_t const* byte_arr) {
        { T{byte_arr} };
        { can_msg.msg_arr } -> std::convertible_to<const uint8_t*>;
        { T::BASE_ID } -> std::convertible_to<uint32_t const>;
    };

    using MRoverCANMsg_t = std::variant<
        BMCModeCmd,
        BMCMotorState,
        BMCResetCmd,
        BMCTargetCmd,
        ESWAck,
        ESWConfigCmd,
        ESWProbe,
        SCIResetCommand,
        SCISensorData,
        SCISensorState
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

    class MRoverCANHandler {
        FDCAN* m_fdcan;
    public:
        MRoverCANHandler() = default;

        explicit MRoverCANHandler(FDCAN* fdcan_driver)
            : m_fdcan{fdcan_driver} {}

        struct SendHandler {
            MRoverCANHandler* m_can_handler;
            uint32_t const m_src_node_id;
            uint32_t const m_dest_node_id;

            SendHandler(MRoverCANHandler* can_handler, uint32_t const src_node_id, uint32_t const dest_node_id)
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
            MRoverCANMsg_t const& message_variant,
            uint32_t const src_node_id,
            uint32_t const dest_node_id
        ) -> void {
            SendHandler sender(this, src_node_id, dest_node_id);
            std::visit(sender, message_variant);
        }

        [[nodiscard]] auto receive() const -> std::optional<MRoverCANMsg_t> {
            FDCAN_RxHeaderTypeDef header;
            uint8_t data_buffer[FDCAN_MAX_FRAME_SIZE];

            if (std::span<uint8_t> const data_span{data_buffer, FDCAN_MAX_FRAME_SIZE}; m_fdcan->receive(&header, data_span)) {
                uint32_t const received_base_id = header.Identifier & ~CAN_NODE_MASK;

                std::size_t const received_size = dlc_to_size(header.DataLength);
                std::span<uint8_t const> const received_data(data_buffer, received_size);

                switch (received_base_id) {
            
                    case (1048576 & ~CAN_NODE_MASK): {
                        return MRoverCANMsg_t{ BMCModeCmd{received_data.data()} };
                    }
                    case (1114112 & ~CAN_NODE_MASK): {
                        return MRoverCANMsg_t{ BMCTargetCmd{received_data.data()} };
                    }
                    case (15728640 & ~CAN_NODE_MASK): {
                        return MRoverCANMsg_t{ ESWConfigCmd{received_data.data()} };
                    }
                    case (1179648 & ~CAN_NODE_MASK): {
                        return MRoverCANMsg_t{ BMCResetCmd{received_data.data()} };
                    }
                    case (1245184 & ~CAN_NODE_MASK): {
                        return MRoverCANMsg_t{ BMCMotorState{received_data.data()} };
                    }
                    case (15794176 & ~CAN_NODE_MASK): {
                        return MRoverCANMsg_t{ ESWProbe{received_data.data()} };
                    }
                    case (15859712 & ~CAN_NODE_MASK): {
                        return MRoverCANMsg_t{ ESWAck{received_data.data()} };
                    }
                    case (6291456 & ~CAN_NODE_MASK): {
                        return MRoverCANMsg_t{ SCISensorData{received_data.data()} };
                    }
                    case (6356992 & ~CAN_NODE_MASK): {
                        return MRoverCANMsg_t{ SCISensorState{received_data.data()} };
                    }
                    case (5308416 & ~CAN_NODE_MASK): {
                        return MRoverCANMsg_t{ SCIResetCommand{received_data.data()} };
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
    class __attribute__((unavailable("enable 'FDCAN' in STM32CubeMX to use mrover::MRoverCANHandler"))) MRoverCANHandler {
    public:
        template<typename... Args>
        explicit MRoverCANHandler(Args&&...) {}
    };
#endif // HAL_FDCAN_MODULE_ENABLED

    template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
    template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;


} // namespace mrover
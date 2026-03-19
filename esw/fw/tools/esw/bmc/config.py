import yaml

# field: (register, addr, dtype, offset, width)
CONFIG_MAP = {
    "can_id": ("CAN_ID", 0x00, int, 0, 8),
    "host_can_id": ("HOST_CAN_ID", 0x01, int, 0, 8),
    # system config bits
    "motor_en": ("SYS_CFG", 0x02, bool, 0, 1),
    "motor_inv": ("SYS_CFG", 0x02, bool, 1, 1),
    "quad_en": ("SYS_CFG", 0x02, bool, 2, 1),
    "quad_phase": ("SYS_CFG", 0x02, bool, 3, 1),
    "stall_en": ("SYS_CFG", 0x02, bool, 4, 1),
    # limit config bits
    "lim_a_en": ("LIMIT_CFG", 0x03, bool, 0, 1),
    "lim_a_active_high": ("LIMIT_CFG", 0x03, bool, 1, 1),
    "lim_a_is_forward": ("LIMIT_CFG", 0x03, bool, 2, 1),
    "lim_a_use_readjust": ("LIMIT_CFG", 0x03, bool, 3, 1),
    "lim_b_en": ("LIMIT_CFG", 0x03, bool, 4, 1),
    "lim_b_active_high": ("LIMIT_CFG", 0x03, bool, 5, 1),
    "lim_b_is_forward": ("LIMIT_CFG", 0x03, bool, 6, 1),
    "lim_b_use_readjust": ("LIMIT_CFG", 0x03, bool, 7, 1),
    # floats
    "quad_cpr": ("QUAD_CPR", 0x04, float, 0, 32),
    "gear_ratio": ("GEAR_RATIO", 0x08, float, 0, 32),
    "rotor_output_ratio": ("ROTOR_OUTPUT_RATIO", 0x0C, float, 0, 32),
    "limit_a_position": ("LIMIT_A_POSITION", 0x10, float, 0, 32),
    "limit_b_position": ("LIMIT_B_POSITION", 0x14, float, 0, 32),
    "max_pwm": ("MAX_PWM", 0x18, float, 0, 32),
    "min_pos": ("MIN_POS", 0x1C, float, 0, 32),
    "max_pos": ("MAX_POS", 0x20, float, 0, 32),
    "min_vel": ("MIN_VEL", 0x24, float, 0, 32),
    "max_vel": ("MAX_VEL", 0x28, float, 0, 32),
    "pos_k_p": ("POS_K_P", 0x2C, float, 0, 32),
    "pos_k_i": ("POS_K_I", 0x30, float, 0, 32),
    "pos_k_d": ("POS_K_D", 0x34, float, 0, 32),
    "pos_k_f": ("POS_K_F", 0x38, float, 0, 32),
    "vel_k_p": ("VEL_K_P", 0x3C, float, 0, 32),
    "vel_k_i": ("VEL_K_I", 0x40, float, 0, 32),
    "vel_k_d": ("VEL_K_D", 0x44, float, 0, 32),
    "vel_k_f": ("VEL_K_F", 0x48, float, 0, 32),
    "stall_current": ("STALL_CURRENT", 0x4C, float, 0, 32),
    "delta_position": ("DELTA_POSITION", 0x50, float, 0, 32),
}


def parse_config(yaml_path):
    with open(yaml_path, "r") as f:
        user_data = yaml.safe_load(f)

    reg_states = {}
    for field, (name, addr, dtype, offset, width) in CONFIG_MAP.items():
        val = user_data.get(field, 0)

        if addr not in reg_states:
            reg_states[addr] = {"name": name, "value": 0, "dtype": dtype}

        if dtype is float:
            reg_states[addr]["value"] = float(val)
        elif dtype is bool:
            if bool(val):
                reg_states[addr]["value"] |= 1 << offset
        else:
            mask = (1 << width) - 1
            reg_states[addr]["value"] |= (int(val) & mask) << offset

    registers = {}
    for addr in sorted(reg_states.keys()):
        info = reg_states[addr]
        val = info["value"]
        registers[info["name"]] = (addr, val)

    return registers

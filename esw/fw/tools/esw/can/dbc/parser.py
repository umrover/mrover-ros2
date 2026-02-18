from datetime import datetime


def get_c_type(signal) -> str:
    if signal.is_float:
        return "float" if signal.length <= 32 else "double"

    prefix = "int" if signal.is_signed else "uint"
    length = signal.length

    if length <= 8:
        return f"{prefix}8_t"
    elif length <= 16:
        return f"{prefix}16_t"
    elif length <= 32:
        return f"{prefix}32_t"
    else:
        return f"{prefix}64_t"


def prepare_context(dbc_db, dbc_name):
    messages = {}
    messages_for_handler = []

    for msg in dbc_db.messages:
        signal_dict = {}
        for sig in msg.signals:
            byte_len = sig.length // 8
            if sig.length % 8 != 0:
                byte_len += 1

            signal_dict[sig.name] = {
                "data_type": get_c_type(sig),
                "bit_length": sig.length,
                "byte_length": byte_len,
            }

        messages[msg.frame_id] = {"name": msg.name, "byte_length": msg.length, "signal_dict": signal_dict}

        messages_for_handler.append(
            {
                "id": int(msg.frame_id),
                "name": msg.name,
            }
        )

    message_types = sorted(list(set(m["name"] for m in messages_for_handler)))

    return {
        "dbc_name": dbc_name,
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "libs": ["cstdlib", "cstdint", "bit", "cstring", "variant", "optional"],
        "message_dict": messages,
        "messages_for_handler": messages_for_handler,
        "message_types": message_types,
    }

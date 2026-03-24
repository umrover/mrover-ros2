from functools import lru_cache

import serial.tools.list_ports as list_ports

from esw import esw_logger


@lru_cache(maxsize=1)
def get_stlinkv3_port():
    ST_VID = 0x0483
    KNOWN_STLINK_PIDS = [0x3748, 0x374B, 0x374E, 0x374F, 0x3753, 0x3754]

    stlink_port = None

    ports = list_ports.comports()
    for port in ports:
        vid = port.vid if port.vid else 0
        pid = port.pid if port.pid else 0
        description = port.description or "N/A"

        if "ST-Link" in description or "STLINK" in description:
            if "V3" in description:
                stlink_port = port.device

        if vid == ST_VID:
            if pid in KNOWN_STLINK_PIDS:
                stlink_port = port.device
            elif "ST-Link" in description:
                stlink_port = port.device

    esw_logger.info(f"Found ST-LINKv3 at {stlink_port}")
    return stlink_port

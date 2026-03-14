import logging
import sys
from pathlib import Path
from time import sleep

import serial

from esw import esw_logger


def _cfg_logger(level=logging.DEBUG):
    logger = logging.getLogger("ST-LINKv3")
    logger.setLevel(level)

    if not logger.handlers:
        ch = logging.StreamHandler(sys.stdout)
        ch.setLevel(logging.DEBUG)
        formatter = logging.Formatter("ST-LINKv3 %(asctime)s [%(levelname)-4s] %(message)s", datefmt="%I:%M:%S%p")
        ch.setFormatter(formatter)
        logger.addHandler(ch)

    return logger


def _process_log_string(logger: logging.Logger, log_message: str):
    LEVEL_MAP = {
        "DEBUG:": logger.debug,
        "INFO:": logger.info,
        "WARNING:": logger.warning,
        "ERROR:": logger.error,
    }
    default_log_func = logger.info

    found_prefix = None
    for prefix in LEVEL_MAP.keys():
        if log_message.startswith(prefix):
            found_prefix = prefix
            break

    if found_prefix:
        log_func = LEVEL_MAP[found_prefix]
        content = log_message[len(found_prefix) :].strip()
        log_func(content)
    else:
        default_log_func(f"{log_message.strip()}")


def stream_serial_data(port_name: Path, baud_rate: int, log_level=logging.DEBUG):
    logger = _cfg_logger(log_level)
    try:
        esw_logger.info(f"Connecting to {port_name}")
        with serial.Serial(
            port=str(port_name),
            baudrate=baud_rate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1,
        ) as ser:
            sleep(1)
            while True:
                raw_line = ser.readline()

                if raw_line:
                    line = raw_line.decode("utf-8", errors="replace")
                    clean_line = line.rstrip()
                    _process_log_string(logger, clean_line)

    except serial.SerialException as e:
        raise e

    except KeyboardInterrupt:
        esw_logger.info("Serial Log Viewer Interrupted, exiting...")

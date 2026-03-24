import argparse
import logging

from esw.stlink import get_stlinkv3_port
from esw.stlink.serial import stream_serial_data


def _logging_level_type(level_string):
    level = getattr(logging, level_string.upper(), None)
    # note stm32 logger does not do CRITICAL logs
    # TODO: should it?
    if level is None or level == logging.NOTSET or level == logging.CRITICAL:
        raise argparse.ArgumentTypeError(
            f"Invalid logging level: '{level_string}'. Must be one of: DEBUG, INFO, WARNING, ERROR"
        )
    return level


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Serial Log Viewer for ST-LINKv3")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Configured Baud Rate of Target Device")
    parser.add_argument(
        "--log-level",
        "-l",
        type=_logging_level_type,
        default=logging.INFO,
        help="Set the logging level (e.g., INFO, DEBUG, WARNING, ERROR). Default: INFO",
    )
    args = parser.parse_args()

    port = get_stlinkv3_port()
    baud = args.baud
    log_level = args.log_level
    stream_serial_data(port, baud, log_level)

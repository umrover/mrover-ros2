import argparse
from pathlib import Path
from time import sleep

from esw import esw_logger
from esw.bmc.config import parse_config
from esw.can.dbc import get_dbc
from esw.can.canbus import CANBus, float2bits


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Send PID Gains to a BMC")
    parser.add_argument(
        "--file",
        "-f",
        type=Path,
        required=True,
        help="Path to BMC Configuration File",
    )
    parser.add_argument(
        "--can",
        "-c",
        type=str,
        required=True,
        help="CAN Network of Device",
    )
    parser.add_argument(
        "--id",
        "-i",
        type=int,
        required=True,
        help="Current CAN ID of Device",
    )
    parser.add_argument(
        "--read",
        "-r",
        action="store_true",
        help="Read configuration from device and save to file",
    )
    args = parser.parse_args()

    # read config file
    cfg = parse_config(args.file)
    node_id = args.id

    # open bus
    with CANBus(get_dbc(dbc_name="CANBus1"), args.can) as bus:
        if args.read:
            # TODO read all configs
            pass
        else:
            # send all configs
            for reg, (addr, val) in cfg.items():
                if "k_" not in str(reg).lower():
                    continue
                esw_logger.info(f"Configured {reg} - ADDR 0x{addr:x} @ {val}")
                val_bits: int
                if isinstance(val, float):
                    val_bits = float2bits(val)
                else:
                    val_bits = val
                bus.send("BMCConfigCmd", {"address": addr, "value": val_bits, "apply": 0x1}, dest_id=node_id)
                sleep(0.1)
                # sleep(50)
                # exit(1)

        sleep(1)
        esw_logger.info("Configuration Complete!")

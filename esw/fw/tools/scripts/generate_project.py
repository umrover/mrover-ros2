import argparse
from pathlib import Path

from esw import esw_logger
from esw.cubemx.generation import generate_project


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate a CubeMX Project")
    parser.add_argument(
        "--mcu",
        "--board",
        "-m",
        "-b",
        type=str,
        required=True,
        help="Target MCU or Board for Generated Project",
    )
    parser.add_argument(
        "--src",
        "-s",
        type=Path,
        required=True,
        help="Path to generated CubeMX Project",
    )
    args = parser.parse_args()

    name = args.src.name
    path = args.src
    mcu = args.mcu

    esw_logger.info(f"Generating project {name} at {path} for target {mcu}")
    generate_project(name, path, mcu)

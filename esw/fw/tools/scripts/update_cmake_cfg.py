import argparse
from pathlib import Path

from esw import esw_logger
from esw.cubemx.cmake import configure_cmake


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Update CMake Toolchain for CubeMX Projects")
    parser.add_argument(
        "--src",
        "-s",
        type=Path,
        required=True,
        help="Path to CubeMX Project",
    )
    parser.add_argument("--root", "-r", type=Path, required=True, help="ESW Root Directory")
    parser.add_argument("--ctx", "-c", type=Path, required=True, help="Template Directory")
    parser.add_argument("--lib", "-l", action="append", default=[], help="Libraries to Include in Generated Project")
    args = parser.parse_args()

    name = args.src.name
    path = args.src
    root = args.root
    ctx = args.ctx
    libs = args.lib

    esw_logger.info(f"Configuring CMake Toolchain for {name} at {path}")
    configure_cmake(name, path, root, ctx, libs)

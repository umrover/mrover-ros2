import argparse
from pathlib import Path

from esw import esw_logger
from esw.cubemx.cmake import configure_clang


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate a clangd Configuration for a Project")
    parser.add_argument(
        "--src",
        "-s",
        type=Path,
        required=True,
        help="Path to generated CubeMX Project",
    )
    parser.add_argument("--ctx", "-c", type=Path, required=True, help="Template Directory")
    args = parser.parse_args()

    path = args.src
    ctx = args.ctx

    esw_logger.info(f"Generating .clangd file for {path}")
    configure_clang(path, ctx)

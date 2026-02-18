import argparse
from pathlib import Path

from esw.can.dbc.hpp_generator import generate_can_header


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Parse dbc files to generate self-contained C++ headers.")
    parser.add_argument("files", nargs="+", help="List of dbc files")
    parser.add_argument("--dest", "-d", type=Path, required=True, help="Output directory")
    parser.add_argument("--ctx", "-c", type=Path, required=True, help="Template directory")

    args = parser.parse_args()

    generate_can_header(args.ctx, args.dest, args.files)

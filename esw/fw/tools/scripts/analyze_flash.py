import re
import sys
from collections import defaultdict


def analyze_map_file(file_path):
    # Regex to match lines with:  Address  Size  Object_File
    # Example: 0x080001dc       0xac ./Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.o
    pattern = re.compile(r"^\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\s+(.+)$")

    file_sizes = defaultdict(int)
    total_flash = 0

    try:
        with open(file_path, "r") as f:
            for line in f:
                match = pattern.match(line)
                if match:
                    addr = int(match.group(1), 16)
                    size = int(match.group(2), 16)
                    obj_file = match.group(3).strip()

                    # Filter for Flash-resident addresses (typically 0x08xxxxxx for STM32)
                    # and exclude lines that aren't actually object files (like section names)
                    if size > 0 and (addr >= 0x08000000 and addr < 0x09000000):
                        # Clean up path to make it readable
                        clean_name = obj_file.split("/")[-1]
                        file_sizes[clean_name] += size
                        total_flash += size

        # Sort by size descending
        sorted_files = sorted(file_sizes.items(), key=lambda x: x[1], reverse=True)

        print(f"{'-' * 60}")
        print(f"{'Object File':<45} | {'Size (Bytes)':<10}")
        print(f"{'-' * 60}")

        for name, size in sorted_files[:25]:  # Show top 25
            print(f"{name:<45} | {size:<10}")

        print(f"{'-' * 60}")
        print(f"Total Flash Accounted for: {total_flash / 1024:.2f} KB")

    except FileNotFoundError:
        print(f"Error: Map file '{file_path}' not found.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python analyze_flash.py <project.map>")
    else:
        analyze_map_file(sys.argv[1])

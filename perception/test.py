#!/usr/bin/env python3

from PIL import Image

with Image.open("in-12.jpg") as img:
    width, height = img.size
    print(f"Width: {width}, Height: {height}")


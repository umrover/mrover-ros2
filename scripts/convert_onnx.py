#!/usr/bin/env python3

from ultralytics import YOLO
import sys

if len(sys.argv) != 2:
    print('usage python3 convert.py [path to pytorch file]')
    exit(1)

# Load the YOLO model
model = YOLO(sys.argv[1])

# Export the model to ONNX format
model.export(format="onnx", imgsz=[640, 640], opset=12)

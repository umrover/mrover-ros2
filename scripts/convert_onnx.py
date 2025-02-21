#!/usr/bin/env python3

import torch
import sys
device = torch.device('cpu')

sys.path.insert(0, '/home/john/ros2_ws/src/mrover')

model = torch.load('/home/john/ros2_ws/src/mrover/data/text.pt', map_location=device, weights_only=False)['model'].float()
torch.onnx.export(model, torch.zeros((1, 3, 640, 640)), 'magic.onnx', opset_version=12)

from __future__ import annotations
from typing import Union

import numpy as np
import torch

from PIL import Image

ImageType = Union[np.ndarray | Image.Image | torch.Tensor, "KeyboardImage"]

Vec2 = tuple[float, float]
Vec3 = tuple[float, float, float]
Vec4 = tuple[float, float, float, float]

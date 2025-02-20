from __future__ import annotations

import torch
import numpy as np

from keyrover.mrovertyping import ImageType
from keyrover.util import to_numpy


def reorder_image_axes(img: ImageType) -> np.ndarray:
    img = to_numpy(img)

    if len(img.shape) == 2:
        return img

    if len(img.shape) != 3:
        raise ValueError(f"reorder_image_axes() requires images with 2 or 3 dimensions, not {img.shape}")

    if img.shape[-1] <= 4:
        return img

    return img.transpose(1, 2, 0)


def normalize(img: np.ndarray | torch.Tensor) -> np.ndarray | torch.Tensor:
    """
    Normalizes an array to the range [0, 1]
    """
    img -= img.min()
    img /= img.max()
    return img


__all__ = ["reorder_image_axes", "normalize"]

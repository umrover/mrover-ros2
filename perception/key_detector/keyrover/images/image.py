from __future__ import annotations
from typing import overload, Literal

import torch
import numpy as np

import cv2
from PIL import Image

from keyrover.paths import DATASETS


ImageType = np.ndarray | Image.Image | torch.Tensor
NormalizationType = Literal["minmax", "unit", "sigmoid", "tanh", "gaussian"]


class KeyboardImage:
    default_folder = "keyboard"

    @overload
    def __init__(self, path: str):
        ...

    @overload
    def __init__(self, image: ImageType):
        ...

    def __init__(self, arg):
        if isinstance(arg, str):
            self._path = f"{DATASETS}/{self.default_folder}/{arg}"

            try:
                self._image = cv2.imread(self._path)
                self._image = cv2.cvtColor(self._image, cv2.COLOR_BGR2RGB)
            except cv2.error:
                raise cv2.error(f"Error opening {self._path}")

        else:
            self._path = None
            self._image = arg

        self._shape = self._image.shape[0], self._image.shape[1]

    def __getitem__(self, args) -> np.ndarray:
        return self._image[args]

    def __eq__(self, other):
        return self._image == other

    def __ne__(self, other):
        return self._image != other

    def __le__(self, other):
        return self._image <= other

    def __lt__(self, other):
        return self._image < other

    def __ge__(self, other):
        return self._image >= other

    def __gt__(self, other):
        return self._image > other

    def __neg__(self):
        return ~self._image

    @overload
    def crop(self, x: int, y: int, w: int, h: int) -> np.array:
        ...

    @overload
    def crop(self, bbox: "BBox") -> np.array:
        ...

    def crop(self, *args) -> np.array:
        if len(args) == 1:
            bbox = args[0]
            return self.crop(*bbox.p1, bbox.width, bbox.height)

        x, y, w, h = map(int, args)
        return self._image[y:y + h, x:x + w]

    def normalize(self, kind="minmax") -> None:
        self._image = self._image.astype("float32")

        if kind == "minmax":
            self._image -= self._image.min()
            self._image /= self._image.max()

        elif kind == "gaussian":
            self._image -= self._image.mean()
            self._image /= self._image.std()

        elif kind == "tanh":
            self._image -= 255 / 2
            self._image = np.tanh(self._image)

        elif kind == "unit":
            self._image -= 255 / 2
            self._image /= 255 / 2

        elif kind == "sigmoid":
            raise NotImplementedError()

        else:
            raise ValueError(f"Unrecognized normalization kind '{kind}'")

    def show(self) -> None:
        from keyrover.plotting import imshow
        imshow(self._image)

    def binarize(self) -> np.ndarray:
        return cv2.cvtColor(self._image, cv2.COLOR_RGB2GRAY)

    path = property(lambda self: self._path)
    image = property(lambda self: self._image)

    shape = property(lambda self: self._shape)
    width = property(lambda self: self._shape[1])
    height = property(lambda self: self._shape[0])

    size = property(lambda self: self._image.size)
    dtype = property(lambda self: self._image.dtype)


ImageType = ImageType | KeyboardImage

__all__ = ["KeyboardImage", "NormalizationType", "ImageType"]

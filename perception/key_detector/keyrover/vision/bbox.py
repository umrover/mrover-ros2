from __future__ import annotations
from typing import overload

import numpy as np

from keyrover.mrovertyping import Vec2


class BBox:
    @overload
    def __init__(self, p1: Vec2, p2: Vec2):
        ...

    @overload
    def __init__(self, centre: Vec2, width: float, height: float):
        ...

    @overload
    def __init__(self, x: float, y: float, width: float, height: float):
        ...

    def __init__(self, *args):
        print(args)
        if len(args) == 2:
            self._p1, self._p2 = args

            self._width = abs(self._p2[0] - self._p1[0])
            self._height = abs(self._p2[1] - self._p1[1])

            self._centre = (self._p1[0] + self._p2[0]) / 2, (self._p1[1] + self._p2[1]) / 2

        elif len(args) == 3:
            self._centre, self._width, self._height = args

            self._width = self._width.item()
            self._height = self._height.item()
            self._centre = [tensor.cpu() for tensor in self._centre]

            self._calculate_p1p2()

        elif len(args) == 4:
            x, y, self._width, self._height = args

            self._width = self._width.item()
            self._height = self._height.item()

            self._centre = (x.item(), y.item())
            self._calculate_p1p2()
        else:
            raise ValueError(f'Invalid args: {args}')

        self._p1 = np.array(self._p1)
        self._p2 = np.array(self._p2)
        self._centre = np.array(self._centre)

    def __repr__(self) -> str:
        return f"BBox({self.centre:.2f}, {self.width:.2f}, {self.height:.2f})"

    def _calculate_p1p2(self):
        self._p1 = (self._centre[0] - self._width / 2, self._centre[1] - self._height / 2)
        self._p2 = (self._centre[0] + self._width / 2, self._centre[1] + self._height / 2)

    # TODO implement using copy or something
    def scale(self, factor) -> BBox:
        return BBox(self._p1 * factor, self._p2 * factor)

    width: float = property(lambda self: self._width)
    height: float = property(lambda self: self._height)

    area: float = property(lambda self: self._width * self._height)

    p1: np.ndarray = property(lambda self: self._p1)
    p2: np.ndarray = property(lambda self: self._p2)
    centre: np.ndarray = property(lambda self: self._centre)

    p1p2: np.ndarray = property(lambda self: (self._p1.astype("int"), self._p2.astype("int")))
    xywh: np.ndarray = property(lambda self: np.array([*self._centre, self._width, self._height], dtype="int"))
    xyxy: np.ndarray = property(lambda self: np.array([*self._p1, *self._p2], dtype="int"))


class LabeledBBox(BBox):
    @overload
    def __init__(self, p1: Vec2, p2: Vec2, label: str):
        ...

    @overload
    def __init__(self, centre: Vec2, width: float, height: float, label: str):
        ...

    @overload
    def __init__(self, x: float, y: float, width: float, height: float, label: str):
        ...

    def __init__(self, *args):
        super().__init__(*args[:-1])
        self._label: str = args[-1]

    def __repr__(self) -> str:
        return f"BBox('{self.label}', {self.centre}, {self.width:.2f}, {self.height:.2f})"

    # TODO remove
    def scale(self, factor) -> LabeledBBox:
        return LabeledBBox(self._p1 * factor, self._p2 * factor, self._label)

    label: str = property(lambda self: self._label)


__all__ = ["BBox", "LabeledBBox"]

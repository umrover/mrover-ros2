from typing import overload


import numpy as np

Point = tuple[float, float]


@overload
def quad_area(x1: float, y1: float, x2: float, y2: float, x3: float, y3: float, x4: float, y4: float) -> int:
    ...


@overload
def quad_area(p1: Point, p2: Point, p3: Point, p4: Point) -> int:
    ...


@overload
def quad_area(rect: np.ndarray) -> int:
    ...


def quad_area(*args) -> float:
    try:
        if len(args) == 8:
            x1, y1, x2, y2, x3, y3, x4, y4 = args

            return 0.5 * abs((x1 * y2 - x2 * y1)
                             + (x2 * y3 - x3 * y2)
                             + (x3 * y4 - x4 * y3)
                             + (x4 * y1 - x1 * y4))

        if len(args) == 4:
            (x1, y1), (x2, y2), (x3, y3), (x4, y4) = args
            return quad_area(x1, y1, x2, y2, x3, y3, x4, y4)

        if len(args) == 1 and isinstance(args[0], np.ndarray):
            x1, y1, x2, y2, x3, y3, x4, y4 = args[0].flatten()
            return quad_area(x1, y1, x2, y2, x3, y3, x4, y4)

    except TypeError:
        pass
    except ValueError:
        pass
    raise ValueError("Unknown format for quad_area")


def aspect_ratio(x1: float, y1: float, x2: float, y2: float, x3: float, y3: float, x4=None, y4=None) -> float:
    w = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
    h = ((x3 - x2) ** 2 + (y3 - y2) ** 2) ** 0.5
    return w / h


__all__ = ["quad_area", "aspect_ratio"]

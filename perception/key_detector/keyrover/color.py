from typing import Sequence

from scipy.spatial import cKDTree
import numpy as np

from keyrover import to_numpy


class Palette:
    def __init__(self, colors: Sequence[tuple[int, int, int]]) -> None:
        self.colors = np.array(colors, dtype="uint8")
        self.cKDTree = cKDTree(self.colors)

    def __iter__(self):
        return iter(self.colors)

    def fit(self, arr: np.ndarray) -> np.ndarray:
        arr = to_numpy(arr)
        indices = self.cKDTree.query(arr, k=1)[1]
        return self.colors[indices]


class NamedPalette(Palette):
    def __init__(self, colors: Sequence[tuple[int, int, int]], names: Sequence[str]) -> None:
        super().__init__(colors)

        self.colors_to_name = {}
        for c, name in zip(colors, names):
            self.colors_to_name[c] = name

    def __getitem__(self, i: str):
        return self.colors_to_name[i]


def image_color(image: np.ndarray, ignore_black: bool = True, reduce="median") -> tuple[int, int, int] | None:
    image = np.vstack(image)

    if ignore_black:
        # noinspection PyUnresolvedReferences
        image = image[(image != 0).any(axis=-1)]

    if image.size == 0:
        return None
    return getattr(np, reduce)(image, axis=0)


__all__ = ["image_color", "Palette"]

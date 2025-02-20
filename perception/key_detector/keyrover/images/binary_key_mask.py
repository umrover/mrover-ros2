from typing import Final

import cv2

from keyrover.vision.bbox import *
from keyrover.math import median_filter

from .image import KeyboardImage


class KeyBinaryMaskImage(KeyboardImage):
    default_folder = "masks"

    def __init__(self, path: str) -> None:
        super().__init__(path.replace("jpg", "png"))

        self._image = self.binarize()
        self._image = (self._image > 1).astype("uint8")  # binarize image

        self._bboxes: Final = self._extract_rects()

    def _extract_rects(self) -> tuple[BBox]:
        bboxes = []

        for contour in cv2.findContours(self._image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]:
            cx, cy, width, height = cv2.boundingRect(contour)
            bboxes.append(BBox((cx, cy), (cx + width, cy + height)))

        areas = map(lambda bbox: bbox.area, bboxes)
        return median_filter(bboxes, statistic=tuple(areas))


__all__ = ["KeyBinaryMaskImage"]

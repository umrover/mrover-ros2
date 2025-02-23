import matplotlib.pyplot as plt

import cv2
import numpy as np

from keyrover.mrovertyping import Vec3, ImageType
from keyrover.util import to_numpy
from keyrover.vision.bbox import *

from .image import imshow


def draw_textbox(img: ImageType, bbox: LabeledBBox, color: Vec3 = (230, 55, 107), thickness: int = 3,
                 font=cv2.FONT_HERSHEY_SIMPLEX, font_size: float = 1.5, draw_text: bool = True) -> np.array:
    """
    Plots a box with text above it
    """

    img = to_numpy(img)
    cv2.rectangle(img, bbox.p1.astype("int"), bbox.p2.astype("int"), color=color, thickness=thickness)

    if draw_text:
        img = cv2.putText(img, bbox.label, bbox.p1.astype("int"),
                          font, font_size, (255, 255, 255), thickness, cv2.LINE_AA)
    return img


def plot_bboxes(img: np.ndarray, boxes: list[LabeledBBox], plot: bool = True, **kwargs) -> np.ndarray | None:
    """
    Plots a YOLO boxes object with labels
    """
    img = to_numpy(img)
    for box in boxes:
        draw_textbox(img, box, **kwargs)

    if not plot:
        return img
    plt.figure()
    imshow(img, ax=plt.gca())

def plot_yolo(results: "ultralytics.engine.results.Results", **kwargs) -> np.ndarray | None:
    bboxes = [LabeledBBox(*box.xywh[0], f"{box.cls} {box.conf}%") for box in results.boxes]
    return plot_bboxes(results.orig_img, bboxes, **kwargs)

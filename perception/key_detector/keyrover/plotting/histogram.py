from typing import Literal

import matplotlib.pyplot as plt

import cv2

from keyrover.mrovertyping import ImageType
from keyrover.util import to_numpy


def imhist(img: ImageType | str,
           ax: plt.Axes | None = None,
           bins: int = 50,
           alpha: float = 0.5,
           histtype: Literal["bar", "barstacked", "step", "stepfilled"] = "stepfilled",
           **kwargs):
    """
    Plots a histogram of the RGB pixels in an image
    """
    if isinstance(img, str):
        img = cv2.imread(img)
    else:
        img = to_numpy(img)

    try:
        r, g, b = img.T
    except ValueError:
        r, g, b, _ = img.T

    if ax is None:
        ax = plt.gca()

    ax.axis("off")
    ax.hist(r.flatten(), color="#fa3c3c", bins=bins, alpha=alpha, histtype=histtype, **kwargs)
    ax.hist(g.flatten(), color="#74db95", bins=bins, alpha=alpha, histtype=histtype, **kwargs)
    ax.hist(b.flatten(), color="#42b3f5", bins=bins, alpha=alpha, histtype=histtype, **kwargs)

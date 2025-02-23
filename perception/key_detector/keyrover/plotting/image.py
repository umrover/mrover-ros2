from __future__ import annotations

import cv2
import numpy as np

from .util import *
from .layout import *

from keyrover.util import to_numpy
from keyrover.mrovertyping import ImageType


def _imshow(img: ImageType, ax: plt.Axes | None) -> plt.Axes:
    """
    Plots an array using matplotlib
    """
    img = to_numpy(img)

    # if the image's data is out of bounds for matplotlib plotting:
    #   - any negative values
    #   - values > 255 and dtype is an integer
    #   - values > 1 and dtype is a float
    if img.min() < 0 or (img.max() > 255 and img.dtype.kind in {"i", "u"}) or (img.max() > 1 and img.dtype.kind == "f"):
        img = normalize(img.astype("float"))

    # matplotlib cannot plot bools, so we convert to uint8
    if img.dtype == bool:
        img = img.astype("uint8")

    # in some cases, the channel comes as the first axis, i.e. (c, w, h)
    # so we reorder to get (w, h, c)
    img = reorder_image_axes(img)

    if img.shape[-1] == 2:
        r, g = cv2.split(img)
        black = np.zeros(r.shape, dtype=img.dtype)
        img = cv2.merge([r, black, g])

    ax.axis("off")
    ax.imshow(img, interpolation="nearest")
    return ax


def imshow(img: ImageType,
           mask: ImageType | None = None,
           ax: plt.Axes | None = None,
           **kwargs) -> plt.Axes | tuple[plt.Axes, ...]:
    """
    Sets up and plots an image using matplotlib
    """

    # this is because when mask is specified, we create a 2-column figure
    # but when we specify, we use an existing subplot
    # hence, both cannot be specified
    if mask is not None and ax is not None:
        raise ValueError("Can't specify both mask and axis!")

    if mask is None:
        if ax is None:
            plt.figure()
            ax = plt.gca()
        return _imshow(img, ax)

    _, (ax1, ax2) = plt.subplots(ncols=2, **kwargs)
    _imshow(img, ax1)
    _imshow(mask, ax2)
    return ax1, ax2


# TODO combine with imshow function
def show_images(images: Sequence[ImageType],
                **kwargs) -> None:
    """
    Plots a grid of images using matplotlib
    """
    _, axes = gridplot(len(images), **kwargs)

    for i, ax in enumerate(axes):
        axes[i].axis("off")
        if i < len(images):
            imshow(images[i], ax=axes[i])

    plt.tight_layout()


def show_channels(img: ImageType) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Shows the RGB channels of an image
    """
    img = to_numpy(img)

    r, g, b = cv2.split(img)
    z = np.zeros(r.shape)

    show_images([img, np.stack([z, z, b]), np.stack([r, z, z]), np.stack([z, g, z])])
    return r, g, b

from typing import Iterable, Sequence

import matplotlib.pyplot as plt


def scatter(points: Iterable[Sequence[float]], ax: None | plt.Axes = None,
            color=None, size=2, **kwargs) -> plt.Axes:
    plt.figure()
    if ax is None:
        ax = plt.gca()

    for (x, y) in points:
        ax.scatter(x, y, c=color, s=size, **kwargs)

    return ax

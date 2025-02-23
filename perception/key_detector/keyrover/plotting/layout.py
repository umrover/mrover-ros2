from typing import Sequence

import matplotlib.pyplot as plt

from keyrover.math import get_median_factors


def gridplot(n: int, **kwargs) -> tuple[plt.Figure, Sequence[plt.Axes]]:
    # creates a grid with the optimal dimensions (more square & few empty spots)
    # that will fit the images
    a, b = get_best_grid(n)
    fig, axes = plt.subplots(a, b, **kwargs)

    if a == b == 1:
        return fig, [axes]

    # flatten array
    axes = [ax for row in axes for ax in row]
    return fig, axes


def get_best_grid(n: int) -> tuple[int, int]:
    """
    Returns the dimensions (a, b) needed to fit n squares.
    Minimizes empty squares (ab — n) and aspect ratio (a — b)
    """
    best_score = float("infinity")
    best_grid = None

    i = n
    while True:
        a, b = get_median_factors(i)
        diff = b - a
        empty = i - n

        if (score := diff + empty) < best_score:
            best_score = score
            best_grid = a, b

        if empty > best_score:
            break
        i += 1

    return best_grid

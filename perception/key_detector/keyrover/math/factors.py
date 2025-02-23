from functools import reduce


def factors(n: int) -> list:
    """
    Returns a list of factors of a number in ascending order
    """
    return sorted(list(set(reduce(list.__add__, ([i, n // i] for i in range(1, int(n ** 0.5) + 1) if n % i == 0)))))


def get_median_factors(n: int) -> tuple[int, int]:
    """
    Returns the median factors of a number.

    For numbers with odd factors, ex. 4 we have [1, 2, 4]. This will return (2, 2)
    For numbers with even factors, ex. 12 we have [1, 2, 3, 4, 6, 12]. This will return (3, 4)
    """
    f = factors(n)
    if len(f) % 2 == 0:
        return f[len(f) // 2 - 1: len(f) // 2 + 1]
    return f[len(f) // 2], f[len(f) // 2]


__all__ = ["factors", "get_median_factors"]

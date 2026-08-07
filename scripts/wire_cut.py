#!/usr/bin/env python3

import sys
from enum import Enum


class WireColor(str, Enum):
    """A wire color, valued by the code used on the command line. Blue and black
    both start with "b", so those two take a second letter.

    This is enum.StrEnum, which needs Python 3.11, while this project supports 3.10.
    """

    RED = "r"
    WHITE = "w"
    BLUE = "bl"
    YELLOW = "y"
    BLACK = "bk"

    @property
    def color_name(self) -> str:
        """The full color name, for printing."""
        return self.name.lower()


def last_wire(wires: list[WireColor], color: WireColor) -> int:
    """Return the 1-indexed position of the rightmost wire of the given color."""
    return len(wires) - wires[::-1].index(color)


def parse_wires(argument: str) -> list[WireColor]:
    """Turn a comma separated string of color codes into a list of wire colors."""
    wires = []

    for code in argument.split(","):
        try:
            wires.append(WireColor(code))
        except ValueError:
            raise ValueError(f'"{code}" is not a valid wire color, expected one of {", ".join(WireColor)}')

    return wires


def wire_to_cut(wires: list[WireColor]) -> int:
    """Return the 1-indexed wire to cut, numbering the wires left to right.

    Rules are evaluated sequentially, so the first one that matches wins.
    """
    count = len(wires)

    if count == 3:
        if wires.count(WireColor.RED) == 0:
            return 2
        if wires[-1] == WireColor.WHITE:
            return 3
        if wires.count(WireColor.BLUE) > 1:
            return last_wire(wires, WireColor.BLUE)
        return 3

    if count == 4:
        if wires.count(WireColor.RED) > 1:
            return last_wire(wires, WireColor.RED)
        if wires[-1] == WireColor.YELLOW and wires.count(WireColor.RED) == 0:
            return 1
        if wires.count(WireColor.BLUE) == 1:
            return 1
        if wires.count(WireColor.YELLOW) > 1:
            return 4
        return 2

    if count == 5:
        if wires[-1] == WireColor.BLACK:
            return 4
        if wires.count(WireColor.RED) == 1 and wires.count(WireColor.YELLOW) > 1:
            return 1
        if wires.count(WireColor.BLACK) == 0:
            return 2
        return 5

    if count == 6:
        if wires.count(WireColor.YELLOW) == 0:
            return 3
        if wires.count(WireColor.YELLOW) == 1 and wires.count(WireColor.WHITE) > 1:
            return 4
        if wires.count(WireColor.RED) == 0:
            return 6
        return 2

    raise ValueError(f"{count} wires is not a valid count, expected 3 to 6")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <wires>, for example {sys.argv[0]} r,bl,y,bk", file=sys.stderr)
        sys.exit(1)

    try:
        wires = parse_wires(sys.argv[1])
        cut = wire_to_cut(wires)
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)

    print(f"{len(wires)} wires: {', '.join(wire.color_name for wire in wires)}")
    print(f"Cut wire {cut} ({wires[cut - 1].color_name})")
    print(f"Note: 1-indexed, left to right")

    # --------------test section---------------
    tests = False

    if tests:
        # One case per rule, in the order the rules are evaluated, so every
        # branch of wire_to_cut is the one that decides its own case.
        cases = [
            # 3 wires
            ("bl,w,bk", 2, "no red"),
            ("r,bl,w", 3, "last wire is white"),
            ("bl,bl,r", 2, "more than one blue, cut the last blue"),
            ("r,bl,bl", 3, "more than one blue, the last blue is also the last wire"),
            ("r,y,bk", 3, "no earlier rule matched, cut the last wire"),
            # 4 wires
            ("r,bl,r,w", 3, "more than one red, cut the last red"),
            ("bl,bl,w,y", 1, "last wire is yellow and there are no reds"),
            ("r,bl,w,bk", 1, "exactly one blue"),
            ("r,y,y,bk", 4, "more than one yellow"),
            ("r,w,bk,bk", 2, "no earlier rule matched, cut the second wire"),
            # 5 wires
            ("r,bl,w,r,bk", 4, "last wire is black"),
            ("r,y,y,w,bl", 1, "exactly one red and more than one yellow"),
            ("r,r,w,bl,bl", 2, "no black"),
            ("bk,r,w,bl,bl", 5, "no earlier rule matched, cut the last wire"),
            # 6 wires
            ("r,bl,w,bk,r,bl", 3, "no yellow"),
            ("y,w,w,bk,r,bl", 4, "exactly one yellow and more than one white"),
            ("y,w,bl,bk,bl,bl", 6, "no red"),
            ("y,w,r,bk,bl,bl", 2, "no earlier rule matched, cut the second wire"),
        ]

        for number, (argument, expected, rule) in enumerate(cases, start=1):
            test_wires = parse_wires(argument)
            print(f"\nTest {number}: {len(test_wires)} wires, {rule}")
            test_cut = wire_to_cut(test_wires)
            if test_cut != expected:
                print(f"bad output: {test_cut}, expected {expected}")
            else:
                print("good")

        # too few wires
        print(f"\nTest {len(cases) + 1}: 2 wires, which is not a valid count")
        try:
            wire_to_cut(parse_wires("r,bl"))
        except ValueError:
            print("caught invalid wire count error")

        # not a wire color
        print(f"\nTest {len(cases) + 2}: b on its own, which is not a valid color")
        try:
            parse_wires("r,b,y,w")
        except ValueError:
            print("caught invalid wire color error")

        # spaces are not part of any code
        print(f"\nTest {len(cases) + 3}: spaces after the commas, which is not allowed")
        try:
            parse_wires("r, bl, y, bk")
        except ValueError:
            print("caught invalid wire color error")

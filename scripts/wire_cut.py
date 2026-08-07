#!/usr/bin/env python3

import sys

# Every wire color, keyed by the code used on the command line. Blue and black
# both start with "b", so those two take a second letter.
WIRE_COLORS = {
    "r": "red",
    "w": "white",
    "bl": "blue",
    "y": "yellow",
    "bk": "black",
}


def last_wire(wires: list[str], color: str) -> int:
    """Return the 1-indexed position of the rightmost wire of the given color."""
    return len(wires) - wires[::-1].index(color)


def parse_wires(argument: str) -> list[str]:
    """Turn a comma separated string of color codes into a list of color names."""
    wires = []

    for code in argument.split(","):
        if code not in WIRE_COLORS:
            raise ValueError(f'"{code}" is not a valid wire color, expected one of {", ".join(WIRE_COLORS)}')
        wires.append(WIRE_COLORS[code])

    return wires


def wire_to_cut(wires: list[str]) -> int:
    """Return the 1-indexed wire to cut, numbering the wires left to right.

    Rules are evaluated sequentially, so the first one that matches wins.
    """
    count = len(wires)

    if count == 3:
        if wires.count("red") == 0:
            return 2
        if wires[-1] == "white":
            return 3
        if wires.count("blue") > 1:
            return last_wire(wires, "blue")
        return 3

    if count == 4:
        if wires.count("red") > 1:
            return last_wire(wires, "red")
        if wires[-1] == "yellow" and wires.count("red") == 0:
            return 1
        if wires.count("blue") == 1:
            return 1
        if wires.count("yellow") > 1:
            return 4
        return 2

    if count == 5:
        if wires[-1] == "black":
            return 4
        if wires.count("red") == 1 and wires.count("yellow") > 1:
            return 1
        if wires.count("black") == 0:
            return 2
        return 5

    if count == 6:
        if wires.count("yellow") == 0:
            return 3
        if wires.count("yellow") == 1 and wires.count("white") > 1:
            return 4
        if wires.count("red") == 0:
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

    print(f"{len(wires)} wires: {', '.join(wires)}")
    print(f"Cut wire {cut} ({wires[cut - 1]})")

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

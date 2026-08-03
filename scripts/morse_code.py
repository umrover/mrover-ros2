#!/usr/bin/env python3

import json
import sys
from pathlib import Path

# Comprehensive dictionary mapping English characters to Morse code
MORSE_DICT = {
    "A": ".-",
    "B": "-...",
    "C": "-.-.",
    "D": "-..",
    "E": ".",
    "F": "..-.",
    "G": "--.",
    "H": "....",
    "I": "..",
    "J": ".---",
    "K": "-.-",
    "L": ".-..",
    "M": "--",
    "N": "-.",
    "O": "---",
    "P": ".--.",
    "Q": "--.-",
    "R": ".-.",
    "S": "...",
    "T": "-",
    "U": "..-",
    "V": "...-",
    "W": ".--",
    "X": "-..-",
    "Y": "-.--",
    "Z": "--..",
    "1": ".----",
    "2": "..---",
    "3": "...--",
    "4": "....-",
    "5": ".....",
    "6": "-....",
    "7": "--...",
    "8": "---..",
    "9": "----.",
    "0": "-----",
    ",": "--..--",
    ".": ".-.-.-",
    "?": "..--..",
    "/": "-..-.",
    "-": "-....-",
    "(": "-.--.",
    ")": "-.--.-",
    " ": "/",
}

# Reverse the dictionary to map Morse code back to English characters
REVERSE_MORSE_DICT = {value: key for key, value in MORSE_DICT.items()}


class MorseCode:

    def __init__(self):
        self.input_str: str = ""
        self.output_str: str = ""

    def encode(self, input_str: str) -> str:
        self.output_str = ""
        self.input_str = input_str.upper()

        for char in self.input_str:
            if not (char in MORSE_DICT):
                raise KeyError(f"{char} is not a key in the Morse Code Dictionary")
            self.output_str += MORSE_DICT[char] + " "

        # remove extra space at the end
        self.output_str = self.output_str[:-1]

        print(f"Encoded string: {self.output_str}")
        return self.output_str

    def decode(self, input_str: str) -> str:
        self.output_str = ""
        self.input_str = input_str

        mc_list: list[str] = self.input_str.split()

        for char in mc_list:
            if not (char in REVERSE_MORSE_DICT):
                raise KeyError(f"{char} is not a key in the Reverse Morse Code Dictionary")
            self.output_str += REVERSE_MORSE_DICT[char]

        print(f"Decoded string: {self.output_str}")
        return self.output_str


def run_config(config_path: Path) -> None:
    """Translate the string in a JSON file and write the result back into that same file.

    The direction is stated explicitly rather than guessed, because every Morse string is
    also a valid plaintext string: "..." is both an ellipsis and the letter S.
    """
    payload = json.loads(config_path.read_text())

    for key in ("direction", "input"):
        if key not in payload:
            raise ValueError(f'"{key}" is missing from {config_path}')

    morse_code_conv = MorseCode()
    direction = payload["direction"]

    if direction == "encode":
        payload["output"] = morse_code_conv.encode(input_str=payload["input"])
    elif direction == "decode":
        payload["output"] = morse_code_conv.decode(input_str=payload["input"])
    else:
        raise ValueError(f'"{direction}" is not a valid direction, expected "encode" or "decode"')

    config_path.write_text(json.dumps(payload, indent=4) + "\n")
    print(f"Wrote output to {config_path}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <json_path>", file=sys.stderr)
        sys.exit(1)

    try:
        run_config(Path(sys.argv[1]))
    except (OSError, ValueError, KeyError) as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)

    # --------------test section---------------
    tests = False

    if tests:
        morse_code_conv = MorseCode()

        letter = "hello world"
        code = ".... . .-.. .-.. --- / .-- --- .-. .-.. -.."

        # test 1
        print("\nTest 1")
        morse_code_conv.encode(input_str=letter)
        if morse_code_conv.output_str != code:
            print(f"bad output: {morse_code_conv.output_str}")
        else:
            print("good")

        # test 2
        print("\nTest 2")
        morse_code_conv.decode(input_str=code)
        if morse_code_conv.output_str != letter.upper():
            print(f"bad output: {morse_code_conv.output_str}")
        else:
            print("good")

        code2 = ".... .-.-.-.-.-.-."

        # test 3
        print("\nTest 3")
        try:
            morse_code_conv.decode(input_str=code2)
        except KeyError:
            print("caught error incorrect input error")

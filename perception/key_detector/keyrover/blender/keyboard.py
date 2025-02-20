from typing import Final

KeyboardX: Final[float] = -0.21919
KeyboardY: Final[float] = -0.081833
KeyboardZ: Final[float] = 0.13053

KeyboardPosition: Final[tuple[float, float, float]] = (KeyboardX, KeyboardY, KeyboardZ)

KeyboardWidth: Final[float] = 2.7986
KeyboardHeight: Final[float] = 0.95583

KeyboardAngle: Final[float] = 0

KeyboardTopLeft: Final[tuple[float, float, float]] = (KeyboardX - KeyboardWidth / 2,
                                                      KeyboardY + KeyboardHeight / 2,
                                                      KeyboardZ)

KeyboardTopRight: Final[tuple[float, float, float]] = (KeyboardX + KeyboardWidth / 2,
                                                       KeyboardY + KeyboardHeight / 2,
                                                       KeyboardZ)

KeyboardBottomLeft: Final[tuple[float, float, float]] = (KeyboardX - KeyboardWidth / 2,
                                                         KeyboardY - KeyboardHeight / 2,
                                                         KeyboardZ)

KeyboardBottomRight: Final[tuple[float, float, float]] = (KeyboardX + KeyboardWidth / 2,
                                                          KeyboardY - KeyboardHeight / 2,
                                                          KeyboardZ)

__all__ = ["KeyboardX", "KeyboardY", "KeyboardZ", "KeyboardPosition", "KeyboardWidth", "KeyboardHeight",
           "KeyboardTopLeft", "KeyboardTopRight", "KeyboardBottomLeft", "KeyboardBottomRight"]

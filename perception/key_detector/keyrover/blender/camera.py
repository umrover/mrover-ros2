from typing import Final

import numpy as np

CameraFx: Final[float] = 888.889
CameraFy: Final[float] = 1000

CameraCx: Final[float] = 888.889
CameraCy: Final[float] = 1000

CameraMatrix: Final[np.ndarray] = np.array([[1, 0, 0],
                                            [0, -1, 0],
                                            [0, 0, -1]])

__all__ = ["CameraFx", "CameraFy", "CameraCx", "CameraCy", "CameraMatrix"]

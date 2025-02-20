from typing import Literal, Final

import torch
from lightning.pytorch.callbacks.model_summary import summarize


DeviceType = Literal["cpu", "cuda", "mps"]

if torch.cuda.is_available():
    device: Final[DeviceType] = "cuda"
elif torch.backends.mps.is_available():
    device: Final[DeviceType] = "mps"
else:
    device: Final[DeviceType] = "cpu"


def identity(*x: torch.Tensor) -> tuple[torch.Tensor, ...] | torch.Tensor:
    if len(x) == 1:
        return x[0]
    return x


__all__ = ["device", "identity", "summarize"]

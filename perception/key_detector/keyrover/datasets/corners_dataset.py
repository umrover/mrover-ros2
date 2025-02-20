import pickle

import torch

from .keyboard_dataset import KeyboardTensorDataset


class KeyboardCornersDataset(KeyboardTensorDataset):
    corners = None

    mean = torch.tensor([[121.73, 304.18, 521.91, 321.19, 518.25, 182.49, 135.07, 166.05]])
    std = torch.tensor([[295.45, 322.01, 291.26, 333.85, 291.21, 329.92, 298.24, 317.73]])

    @staticmethod
    def corners_from_filename(filename):
        if KeyboardCornersDataset.corners is None:
            with open("datasets/corners/corners.pkl", "rb") as f:
                KeyboardCornersDataset.corners = pickle.load(f)

        frame = int(filename.split("_")[1])
        return KeyboardCornersDataset.corners[frame - 1]

    @staticmethod
    def denormalize(arr: torch.Tensor) -> torch.Tensor:
        return arr * KeyboardCornersDataset.std.to(arr.device) + KeyboardCornersDataset.mean.to(arr.device)

    _target = corners_from_filename


__all__ = ["KeyboardCornersDataset"]

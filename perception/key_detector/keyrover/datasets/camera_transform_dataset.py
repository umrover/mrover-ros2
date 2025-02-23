from typing import Sequence

import os
import pickle

from tqdm.notebook import tqdm
from multiprocessing import Pool

from PIL import Image

import torch
from torchvision.transforms import v2 as transforms

from .keyboard_dataset import KeyboardDataset


class KeyboardCameraTransformDataset(KeyboardDataset):
    with open("blender/normalized_camera_data.bin", "rb") as f:
        camera_data = pickle.load(f)

    def __init__(self, paths: Sequence[str], size: tuple[float, float] = None):

        self._resize = identity if size is None else transforms.Resize(size)

        with Pool() as p:
            images = list(tqdm(p.imap(self._get_img, paths), total=len(paths)))

        self._images, self._targets = zip(*images)
        super().__init__()

    def __getitem__(self, idx) -> tuple[torch.Tensor, torch.Tensor]:
        img = self._images[idx]
        target = self._targets[idx]
        return self._input_augmentations(img), target

    def _get_img(self, path: str) -> tuple[torch.Tensor, torch.Tensor]:
        img = Image.open(path)
        img = self._resize(self._to_image(img))

        frame = int(os.path.basename(path).split("_")[1]) - 1
        target = self.camera_data[frame]

        return img, target


__all__ = ["KeyboardCameraTransformDataset"]

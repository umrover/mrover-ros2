from __future__ import annotations
from typing import Sequence, Literal, Callable

from multiprocessing import Pool
from tqdm import tqdm

import random
from functools import partial

import cv2

import torch
from torch.utils.data import Dataset
from torchvision.transforms import v2 as transforms

from keyrover.datasets.util import *
from keyrover.util import to_tensor
from keyrover.vision import identity

from keyrover.images import KeyboardImage, ImageType


class KeyboardDataset(Dataset):
    _target: Callable[[str, ...], ImageType] = None

    def __init__(self, filenames: Sequence[str], size: tuple[int, int], version: str | None = None, **kwargs):
        self._resize = transforms.Resize(size)
        self._version: str | None = version

        with Pool() as p:
            self._targets = tqdm(p.imap(partial(self._target, **kwargs), filenames), total=len(filenames))
            self._targets = list(map(self._target_to_tensor, self._targets))

            self._images = tqdm(p.imap(KeyboardImage, filenames), total=len(filenames))
            self._images = [self._resize((to_tensor(image))) for image in self._images]

        self._transforms = identity
        self._target_augmentations = identity

        self._input_augmentations = None
        self.set_input_augmentations([])

    def __len__(self) -> int:
        return len(self._images)

    def __getitem__(self, item) -> tuple[torch.Tensor, torch.Tensor]:
        image = self._images[item]
        target = self._targets[item]

        image, target = self._transforms(image, target)

        image = self._input_augmentations(image)
        target = self._target_augmentations(target)

        return image, target

    def _target_to_tensor(self, arr: ImageType) -> torch.Tensor:
        raise NotImplementedError()

    @classmethod
    def load(cls, version: str, n: int = None, **kwargs) -> tuple[KeyboardDataset, KeyboardDataset, KeyboardDataset]:
        image_paths = get_dataset_paths(version=version)
        print(f'Image Paths {image_paths}')
        if n is not None:
            image_paths = image_paths[:n]

        train_paths, test_paths, valid_paths = split_train_test_valid(image_paths, 0.9, 0.1)

        train_dataset = cls(train_paths, version=version, **kwargs)
        valid_dataset = cls(valid_paths, version=version, **kwargs)
        test_dataset = cls(test_paths, version=version, **kwargs)

        return train_dataset, valid_dataset, test_dataset

    def set_transforms(self, val: Sequence[transforms.Transform]) -> None:
        if len(val) == 0:
            self._transforms = identity
        else:
            self._transforms = transforms.Compose(val)

    def set_input_augmentations(self, val: list[transforms.Transform],
                                norm_params: None | dict | Literal["default"] = "default") -> None:
        val.insert(0, transforms.ToDtype(torch.float32, scale=True))

        if norm_params == "default" and self._version is not None:
            norm_params = get_dataset_norm_params(version=self._version)

        if norm_params is not None:
            val.append(transforms.Normalize(norm_params["mean"], norm_params["std"]))

        self._input_augmentations = transforms.Compose(val)
        print(f'Transforms being used {self._input_augmentations}')

    def set_target_augmentations(self, val: Sequence[transforms.Transform]) -> None:
        self._target_augmentations = transforms.Compose(val)

    def random_img(self) -> tuple[torch.Tensor, torch.Tensor]:
        return self[random.randint(0, len(self) - 1)]

    def load_image(self, filename) -> torch.Tensor:
        img = cv2.imread(filename)
        img = self._resize((to_tensor(img)))
        return self._input_augmentations(img)

    def cvt_image(self, img):
        img = self._resize((to_tensor(img)))
        return self._input_augmentations(img)


class KeyboardImageDataset(KeyboardDataset):
    def _target_to_tensor(self, arr: ImageType) -> torch.Tensor:
        return self._resize(to_tensor(arr))


class KeyboardTensorDataset(KeyboardDataset):
    def _target_to_tensor(self, arr: ImageType) -> torch.Tensor:
        return to_tensor(arr)


__all__ = ["KeyboardDataset", "KeyboardImageDataset", "KeyboardTensorDataset"]

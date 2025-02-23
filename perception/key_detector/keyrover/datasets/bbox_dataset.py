import os.path
import pickle
from typing import Iterable

import cv2 as cv
import torch

from tqdm.notebook import tqdm

from torchvision.transforms import v2 as transforms

from keyrover import Image, RAW_DATASET, reorder_image_axes, imshow
from .keyboard_dataset import KeyboardDataset


class KeyboardBBoxDataset(KeyboardDataset):
    def __init__(self, paths: Iterable[str]):
        super().__init__()

        self._targets = []

        with open(f"{RAW_DATASET}/regions.pkl", "rb") as file:
            targets = pickle.load(file)

        f = transforms.Compose([
            transforms.ToImage(),
            transforms.ToDtype(torch.float32, scale=True),
        ])

        to_xyxy = transforms.ConvertBoundingBoxFormat("XYXY")

        for path in tqdm(paths):
            img = Image.open(path)
            self._images.append(f(img))
            self._targets.append(to_xyxy(targets[os.path.basename(path)]))

    def show(self, idx: int) -> None:
        img, target = self[idx]
        img = reorder_image_axes(img.numpy()).copy()

        for quad in target["boxes"]:
            x1, y1, x2, y2 = quad.numpy()
            cv.rectangle(img, (x1, y1), (x2, y2), (1.0, 0, 0), 2)

        poly = 0
        factor = int(255 / len(target["masks"]))

        for i, mask in enumerate(target["masks"]):
            poly += i * factor * mask

        imshow(img, poly)

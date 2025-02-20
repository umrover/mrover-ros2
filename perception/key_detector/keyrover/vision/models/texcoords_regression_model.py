from typing import Literal

import torch
import numpy as np

from .keyboard_model import *
from keyrover.util import to_numpy

import segmentation_models_pytorch as smp


class TexcoordsRegressionModel(KeyboardModel):
    _save_path = "texcoords"

    def __init__(self,
                 arch: Literal["unet", "unetplusplus"],
                 encoder_name: str,
                 in_channels: int, out_classes: int, lr: float,
                 **kwargs) -> None:
        super().__init__()

        self.model = smp.create_model(arch, encoder_name, in_channels=in_channels, classes=out_classes, **kwargs)
        self.loss_fn = torch.nn.MSELoss()

        self.lr = lr
        self.save_hyperparameters()

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        return self.model(image)

    def predict(self, image: torch.Tensor) -> np.ndarray:
        image = image.to(self.device)
        if len(image.shape) == 3:
            image = image.unsqueeze(0)

        with torch.no_grad():
            pred = to_numpy(self(image))

        if len(pred) == 1:
            return pred[0,]
        return pred

    def _step(self, batch: tuple[torch.Tensor, torch.Tensor], stage: str) -> float:
        image, truth = batch
        prediction = self.forward(image)

        loss = self.loss_fn(prediction, truth)
        self.log(f"{stage}_loss", loss)
        return loss


__all__ = ["TexcoordsRegressionModel"]

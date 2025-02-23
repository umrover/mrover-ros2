from typing import Literal

import numpy as np
import torch

from .keyboard_model import *
import segmentation_models_pytorch as smp


class BinarySegmentationModel(KeyboardModel):
    _save_path = "binary-segmentation"

    def __init__(self,
                 arch: Literal["unet", "unetplusplus"],
                 encoder_name: str,
                 in_channels: int, out_classes: int, lr: float,
                 loss: Literal["Jaccard", "Dice"] = "Jaccard",
                 **kwargs) -> None:
        super().__init__()

        self.model = smp.create_model(arch, encoder_name, in_channels=in_channels, classes=out_classes, **kwargs)

        if loss == "Jaccard":
            self.loss_fn = smp.losses.JaccardLoss(smp.losses.BINARY_MODE)
        elif loss == "Dice":
            self.loss_fn = smp.losses.DiceLoss(smp.losses.BINARY_MODE, from_logits=True)
        else:
            raise ValueError("Unknown loss function")

        self.lr = lr
        self.save_hyperparameters()

    def forward(self, image: torch.Tensor) -> torch.Tensor:
        return self.model(image)

    def predict(self, image: torch.Tensor) -> np.ndarray:
        image = image.to(self.device)
        if len(image.shape) == 3:
            image = image.unsqueeze(0)

        with torch.no_grad():
            pred = self(image).cpu().numpy()

        if len(pred) == 1:
            return pred[0, 0]
        return pred[:, 0]

    def _step(self, batch: tuple[torch.Tensor, torch.Tensor], stage: str) -> float:
        image, truth = batch
        prediction = self.forward(image)

        loss = self.loss_fn(prediction, truth.int())
        self.log(f"{stage}_loss", loss)
        return loss


__all__ = ["BinarySegmentationModel"]

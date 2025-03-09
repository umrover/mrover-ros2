import torch
from torchvision import models

import numpy as np

from keyrover.datasets import KeyboardCornersDataset
from keyrover.math.bilinear import InterpolateQuad
from keyrover.util import to_numpy
from .keyboard_model import KeyboardModel


class CornersRegressionModel(KeyboardModel):
    _save_path = "corner-prediction"

    def __init__(self, lr: float | None = None) -> None:
        super().__init__()

        self.model = models.resnet18(weights=models.ResNet18_Weights.DEFAULT)

        self.model.fc = torch.nn.Sequential(
            torch.nn.Linear(self.model.fc.in_features, 256),
            torch.nn.ReLU(),
            torch.nn.Linear(256, 64),
            torch.nn.ReLU(),
            torch.nn.Linear(64, 16),
            torch.nn.ReLU(),
            torch.nn.Linear(16, 8),
        )

        self.loss_fn = torch.nn.MSELoss()

        self.lr = lr
        self.save_hyperparameters()

    def predict(self, image: torch.Tensor, mask: bool = True) -> np.ndarray:
        image = image.to(self.device)
        if len(image.shape) == 3:
            print(f'AWOIDOWIJDOIAWOIDJOAWIJDOIJAWOIDJWOAIJDOIWJA*WD*WDU*AWD*UAW*DUA*WU')
            image = image.unsqueeze(0)
        print(f'image shape {image.shape}')

        with torch.no_grad():
            pred = self.forward(image, mask=mask)

        print(f'pred {pred}')
        pred = KeyboardCornersDataset.denormalize(pred)

        transform = InterpolateQuad(len(image), width=640, height=480, device=self.device)
        pred = to_numpy(transform(pred))

        if len(pred) == 1:
            pred = pred[0,]
        return pred.transpose((1, 2, 0))

    def forward(self, image: torch.Tensor, mask: bool = True) -> torch.Tensor:
        return self.model(image)

    def _step(self, batch: tuple[torch.Tensor, torch.Tensor], stage: str) -> float:
        image, target = batch
        predictions = self(image)

        loss = self.loss_fn(predictions, target)
        self.log(f"{stage}_loss", loss)
        return loss

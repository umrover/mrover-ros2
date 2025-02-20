from keyrover import MODELS_PATH
import torch
import lightning as pl


class KeyboardModel(pl.LightningModule):
    _save_path = None

    def _step(self, batch: tuple[torch.Tensor, torch.Tensor], stage: str) -> float:
        raise NotImplementedError()

    def training_step(self, batch: tuple[torch.Tensor, torch.Tensor], batch_idx: int) -> float:
        return self._step(batch, "train")

    def validation_step(self, batch: tuple[torch.Tensor, torch.Tensor], batch_idx: int) -> float:
        return self._step(batch, "val")

    def test_step(self, batch: tuple[torch.Tensor, torch.Tensor], batch_idx: int) -> float:
        return self._step(batch, "test")

    def configure_optimizers(self) -> dict:
        optimizer = torch.optim.AdamW(self.parameters(), lr=self.lr)
        return {"optimizer": optimizer}

    def save(self, filename: str):
        torch.save(self.state_dict(), f"models/{self._save_path}/{filename}")

    @classmethod
    def load(cls, filename: str):
        model = cls()
        model.load_state_dict(torch.load(f"{MODELS_PATH}/{cls._save_path}/{filename}", weights_only=True, map_location=torch.device('cpu')))
        return model


__all__ = ["KeyboardModel"]

from typing import Sequence
import torch

TensorType = torch.Tensor | Sequence[float]


class BatchedLinearAlgebra:
    def __init__(self, batch_size: int, device):
        self.batch_size = batch_size
        self.device = device

        self.ones = torch.ones(self.batch_size, dtype=torch.float32, device=self.device)
        self.zeros = torch.zeros(self.batch_size, dtype=torch.float32, device=self.device)

    def _to_tensor(self, p: TensorType, keepdims=False) -> torch.Tensor:
        if isinstance(p, torch.Tensor):
            return p.to(self.device)

        return torch.tensor(p if keepdims else [p], device=self.device, dtype=torch.float32)


__all__ = ["BatchedLinearAlgebra", "TensorType"]

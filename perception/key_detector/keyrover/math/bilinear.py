from typing import overload

import torch

import numpy as np

from .linalg import BatchedLinearAlgebra, TensorType
from .projection import BatchedProjectionMatrix
from .homogenous import HomogenousChangeOfBasis

Vec3 = tuple[float, float, float]


class InterpolateQuad(BatchedLinearAlgebra):
    def __init__(self, batch_size: int, device, width: int, height: int):
        super().__init__(batch_size, device)

        self.coordinates = self._coordinates_mesh(width=width, height=height, resolution=2)

        change_of_basis = HomogenousChangeOfBasis(1, device)
        self.change_of_basis = HomogenousChangeOfBasis(batch_size, device)

        uv1 = torch.tensor([[0], [0]], dtype=torch.float32)
        uv2 = torch.tensor([[1], [0]], dtype=torch.float32)
        uv3 = torch.tensor([[1], [1]], dtype=torch.float32)
        uv4 = torch.tensor([[0], [1]], dtype=torch.float32)

        self.UV_basis = change_of_basis(uv1, uv2, uv3, uv4)

    @overload
    def __call__(self, corners: torch.Tensor) -> torch.Tensor:
        ...

    @overload
    def __call__(self, p1: torch.Tensor, p2: torch.Tensor, p3: torch.Tensor, p4: torch.Tensor) -> torch.Tensor:
        ...

    def __call__(self, *args: torch.Tensor) -> torch.Tensor:
        if len(args) == 4:
            p1, p2, p3, p4 = args
        else:
            arg = args[0]
            if arg.dim() == 1:
                arg = arg.unsqueeze(0)
            arg = arg.T

            p1 = arg[:2]
            p2 = arg[2:4]
            p3 = arg[4:6]
            p4 = arg[6:]

        XY_basis = self.change_of_basis(p1, p2, p3, p4)
        C = self.UV_basis @ torch.inverse(XY_basis)

        """
        [u, v, z] = C @ [x, y, 1]
        result = [u / z, y / z]
         """
        result = torch.einsum('bij, yxj -> byxi', C, self.coordinates)
        result = result[:, :, :, :3] / result[:, :, :, -1:]

        # filter any points outside the valid range
        #result[(result.max(dim=-1)[0] > 1)] = 0
        #result[result.min(dim=-1)[0] < 0] = 0

        # reorder dimensions from (batch, y, x, channel) to (batch, channel, y, x)
        result = torch.einsum('byxc -> bcyx', result)
        return result

    def _coordinates_mesh(self, width: int, height: int, resolution: float) -> torch.Tensor:
        x = np.linspace(0, width, int(width / resolution))
        y = np.linspace(0, height, int(height // resolution))
        xx, yy = np.meshgrid(x, y)
        ones = np.ones(xx.shape)

        mesh = np.stack((xx, yy, ones), axis=-1)
        return torch.tensor(mesh, dtype=torch.float32, device=self.device)


class InverseBilinear(BatchedLinearAlgebra):
    def __init__(self, batch_size: int, device, width: int, height: int,
                 p1: Vec3, p2: Vec3, p3: Vec3, p4: Vec3):
        """
        Args:
            batch_size:
            device: "cuda" or "cpu" or "mps"
            width: in pixels
            height: in pixels
            p1: (x, y) in world coordinates
            p2: (x, y) in world coordinates
            p3: (x, y) in world coordinates
            p4: (x, y) in world coordinates
        """
        super().__init__(batch_size, device)

        self.projection_matrix = BatchedProjectionMatrix(batch_size, device)
        self.quad_interpolation = InterpolateQuad(batch_size, device, width, height)

        self.p1 = torch.tensor([*p1, 1], dtype=torch.float32, device=self.device)
        self.p2 = torch.tensor([*p2, 1], dtype=torch.float32, device=self.device)
        self.p3 = torch.tensor([*p3, 1], dtype=torch.float32, device=self.device)
        self.p4 = torch.tensor([*p4, 1], dtype=torch.float32, device=self.device)

    def __call__(self, alpha: TensorType, beta: TensorType, gamma: TensorType, position: TensorType) -> torch.Tensor:
        corners = self.project_corners(alpha, beta, gamma, position)
        return self.quad_interpolation(*corners)

    def project_corners(self, alpha: TensorType, beta: TensorType, gamma: TensorType, position: TensorType) \
            -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        projection = self.projection_matrix(alpha, beta, gamma, position)
        return projection(self.p1), projection(self.p2), projection(self.p3), projection(self.p4)


__all__ = ["InverseBilinear", "InterpolateQuad"]

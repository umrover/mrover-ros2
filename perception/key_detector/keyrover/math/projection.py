import torch

from keyrover import CameraMatrix
from .linalg import BatchedLinearAlgebra, TensorType
from .rotation import RotationMatrix


class PerspectiveProjection:
    def __init__(self, matrix: torch.Tensor):
        self.matrix = matrix

    def __call__(self, v: torch.Tensor) -> torch.Tensor:
        """
        [x', y', z'] = P @ [x, y, 1]
        return [x' / z', y' / z']
        """
        v = self.matrix @ v
        return (v[:, :2] / v[:, -1:]).T


class BatchedProjectionMatrix(BatchedLinearAlgebra):
    def __init__(self, batch_size: int, device):
        super().__init__(batch_size, device)

        fx = 888.889  # TODO calculation of fx, fy from blender camera data
        fy = 1000
        self.K = torch.tensor([[fx, 0, 320],
                               [0, fy, 240],
                               [0, 0, 1]], dtype=torch.float32, device=self.device)

        self.R_to_Blender = torch.tensor(CameraMatrix, dtype=torch.float32, device=self.device)

        self.rotation_matrix = RotationMatrix(batch_size, device)

    def __call__(self,
                 alpha: TensorType, beta: TensorType, gamma: TensorType, position: TensorType) -> PerspectiveProjection:
        matrix = self.projection_matrix(alpha, beta, gamma, position)
        return PerspectiveProjection(matrix)

    def extrinsic_matrix(self, alpha, beta, gamma, position):
        """
        position = [x, y, z]

        E = [R | -R @ position]
        """
        R = self.rotation_matrix(alpha, beta, gamma)

        position = position.unsqueeze(axis=-1)
        T = -R @ position

        R = self.R_to_Blender @ R
        T = self.R_to_Blender @ T
        return torch.concat([R, T], dim=-1)

    def intrinsic_matrix(self, fx, fy, cx, cy):
        """
        K = [[fx, 0, cx],
             [0, fy, cy],
             [0,  0,  1]]
        """
        K1 = torch.stack([fx, self.zeros, cx], dim=1)
        K2 = torch.stack([self.zeros, fy, cy], dim=1)
        K3 = torch.stack([self.zeros, self.zeros, self.ones], dim=1)
        return torch.stack([K1, K2, K3], dim=1)

    def projection_matrix(self,
                          alpha: TensorType, beta: TensorType, gamma: TensorType, position: TensorType) -> torch.Tensor:
        """
        P = K @ E
        """
        assert type(alpha) is type(beta), "alpha, beta, & gamma must be same dtype"
        assert type(alpha) is type(gamma), "alpha, beta, & gamma must be same dtype"
        keepdims = not isinstance(alpha, (float, int))

        alpha = self._to_tensor(alpha, keepdims=keepdims)
        beta = self._to_tensor(beta, keepdims=keepdims)
        gamma = self._to_tensor(gamma, keepdims=keepdims)
        position = self._to_tensor(position, keepdims=keepdims)

        E = self.extrinsic_matrix(alpha, beta, gamma, position)
        return self.K @ E


__all__ = ["BatchedProjectionMatrix", "PerspectiveProjection"]

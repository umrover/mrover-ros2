import torch

from .linalg import BatchedLinearAlgebra


class RotationMatrix(BatchedLinearAlgebra):
    def __call__(self, alpha: torch.Tensor, beta: torch.Tensor, gamma: torch.Tensor) -> torch.Tensor:
        return self.rotation_matrix(alpha, beta, gamma)

    def alpha_rotation_matrix(self, alpha) -> torch.Tensor:
        """
        Rx = [[1, 0, 0],
              [0, cos(α), sin(α)],
              [0, -sin(α), cos(α)]]
        """
        cos_alpha = torch.cos(alpha)
        sin_alpha = torch.sin(alpha)

        Rx1 = torch.stack([self.ones, self.zeros, self.zeros], dim=1)
        Rx2 = torch.stack([self.zeros, cos_alpha, sin_alpha], dim=1)
        Rx3 = torch.stack([self.zeros, -sin_alpha, cos_alpha], dim=1)
        return torch.stack([Rx1, Rx2, Rx3], dim=1)

    def beta_rotation_matrix(self, beta: torch.Tensor) -> torch.Tensor:
        """
        Ry = [[cos(β), 0, -sin(β)],
              [0, 1, 0],
              [sin(β), 0, cos(β)]]
        """
        cos_beta = torch.cos(beta)
        sin_beta = torch.sin(beta)

        Ry1 = torch.stack([cos_beta, self.zeros, -sin_beta], dim=1)
        Ry2 = torch.stack([self.zeros, self.ones, self.zeros], dim=1)
        Ry3 = torch.stack([sin_beta, self.zeros, cos_beta], dim=1)
        return torch.stack([Ry1, Ry2, Ry3], dim=1)

    def gamma_rotation_matrix(self, gamma: torch.Tensor) -> torch.Tensor:
        """
        Rz = [[cos(γ), sin(γ), 0],
              [-sin(γ),  cos(γ), 0],
              [0,  0, 1]]
        """
        cos_gamma = torch.cos(gamma)
        sin_gamma = torch.sin(gamma)

        Rz1 = torch.stack([cos_gamma, sin_gamma, self.zeros], dim=1)
        Rz2 = torch.stack([-sin_gamma, cos_gamma, self.zeros], dim=1)
        Rz3 = torch.stack([self.zeros, self.zeros, self.ones], dim=1)
        return torch.stack([Rz1, Rz2, Rz3], dim=1)

    def rotation_matrix(self, alpha: torch.Tensor, beta: torch.Tensor, gamma: torch.Tensor) -> torch.Tensor:
        """
        R = Rx @ Ry @ Rz
        """
        Rx = self.alpha_rotation_matrix(alpha)
        Ry = self.beta_rotation_matrix(beta)
        Rz = self.gamma_rotation_matrix(gamma)

        return Rx @ Ry @ Rz


__all__ = ["RotationMatrix"]

import torch

from .linalg import BatchedLinearAlgebra, TensorType


class HomogenousChangeOfBasis(BatchedLinearAlgebra):
    """
    Given 4 (x, y) coordinates, this finds a homogenous change of basis matrix
    that can be used to convert to the standard basis

    See https://math.stackexchange.com/questions/296794/finding-the-transform-matrix-from-4-projected-points-with-javascript
    """

    def __call__(self, p1: TensorType, p2: TensorType, p3: TensorType, p4: TensorType) -> torch.Tensor:
        p1 = self._to_tensor(p1)
        p2 = self._to_tensor(p2)
        p3 = self._to_tensor(p3)
        p4 = self._to_tensor(p4)

        """
        X = [x4, y4, 1]
        """
        X = torch.stack([p4[0], p4[1], self.ones], dim=1)
        X = X.unsqueeze(dim=-1)

        """
        M = [[x1, x2, x3],
             [y1, y2, y3],
             [ 1,  1,  1]]
         """
        M1 = torch.stack([p1[0], p2[0], p3[0]], dim=1)
        M2 = torch.stack([p1[1], p2[1], p3[1]], dim=1)
        M3 = torch.stack([self.ones, self.ones, self.ones], dim=1)
        M = torch.stack((M1, M2, M3), dim=1)
        print(f'M {M}')

        M_inv = torch.inverse(M)
        print(f'M_INV {M_inv}')

        """
        H = M @ X = [λ, μ, τ]
        """
        H = M_inv @ X
        H = H.squeeze(dim=-1)
        print(f'H {H}')

        """
        A = [[λ * x1, μ * x2, τ * x3],
             [λ * y1, μ * y2, τ * y3],
             [     λ,      μ,     τ]]
        """
        A = torch.einsum('bij, bj -> bij', M, H)

        print(f'A {A}')
        return A


__all__ = ["HomogenousChangeOfBasis"]

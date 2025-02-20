from keyrover.images import KeyboardImage
from keyrover.mrovertyping import *


def describe(arr) -> None:
    print(
        f"""{arr.__class__} ({arr.dtype}, shape={arr.shape})
        Min: {arr.min()}
        Max: {arr.max()}
        Mean: {arr.mean()}""")


def to_numpy(arr, convert_bool: bool = False) -> np.ndarray:
    if isinstance(arr, (map, filter, zip)):
        return np.array(tuple(arr))

    if isinstance(arr, (Image.Image, tuple, list)):
        arr = np.array(arr)

    elif isinstance(arr, torch.Tensor):
        arr = arr.detach().cpu().numpy()

    elif isinstance(arr, KeyboardImage):
        return arr.image

    elif not isinstance(arr, np.ndarray):
        raise TypeError(f"Can't convert unknown type {type(arr)} to numpy array")

    if convert_bool and arr.dtype == bool:
        arr = arr.astype("uint8")
    return arr


def to_pillow(img: ImageType) -> Image.Image:
    if isinstance(img, Image.Image):
        return img
    return Image.fromarray(to_numpy(img))


def to_tensor(img: ImageType) -> torch.Tensor:
    if isinstance(img, torch.Tensor):
        return img
    img = torch.Tensor(to_numpy(img))

    if img.dim() == 1:
        return img
    elif img.dim() == 2:  # no channel dimension
        return img.unsqueeze(0)
    return img.permute(2, 0, 1)


def to_int(vec: tuple[float, ...]) -> tuple[int, ...]:
    return tuple(map(int, vec))


__all__ = ["to_int", "to_tensor", "to_pillow", "to_numpy", "describe",
           "Vec2", "Vec3", "Vec4", "ImageType"]

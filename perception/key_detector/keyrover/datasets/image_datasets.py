from keyrover.images.binary_key_mask import KeyBinaryMaskImage
from keyrover.images.texcoord import NormalizedTexcoordImage

from .keyboard_dataset import KeyboardImageDataset


class KeyboardBinaryMaskDataset(KeyboardImageDataset):
    _target = KeyBinaryMaskImage


class KeyboardTexcoordDataset(KeyboardImageDataset):
    _target = NormalizedTexcoordImage


__all__ = ["KeyboardBinaryMaskDataset", "KeyboardTexcoordDataset"]

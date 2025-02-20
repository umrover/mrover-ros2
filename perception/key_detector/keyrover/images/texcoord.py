from typing import Iterable, overload

from keyrover.color import image_color
from keyrover.vision.bbox import LabeledBBox
from keyrover.util import to_numpy

from .image import KeyboardImage, NormalizationType, ImageType
from .key_mask import KeyMaskImage


TextureCoordinate = tuple[float, float]


class TexcoordImage(KeyboardImage):
    default_folder = "texcoords"

    @overload
    def __init__(self, texcoords: ImageType, bboxes: Iterable[LabeledBBox], reduce: str = "median"):
        ...

    @overload
    def __init__(self, path: str, reduce: str = "median"):
        ...

    def __init__(self, *args, reduce: str = "median") -> None:
        if len(args) == 1:
            path = args[0]
            super().__init__(path.replace("jpg", "png"))
            self._bboxes = KeyMaskImage(path)

        else:
            super().__init__(to_numpy(args[0]))
            self._bboxes = args[1]

        if isinstance(self, NormalizedTexcoordImage):
            self.normalize(self._normalization)

        self._texcoords = self._extract_texcoords(reduce=reduce)

    def _extract_texcoords(self, reduce: str) -> dict[str, TextureCoordinate]:
        texcoords = {}

        for key in self._bboxes:
            crop = self.crop(key)

            if len(crop) == 0:
                raise ValueError(f"bounding box likely out-of-bounds, {key}")

            if (color := image_color(crop, reduce=reduce)) is None:
                print(f"WARNING: empty bounding box {key}")
                continue

            r, g, _ = color
            texcoords[key.label] = (r, g)

        return texcoords

    def scatter(self, color="#ccc", **kwargs):
        from keyrover.plotting import scatter
        return scatter(self._texcoords.values(), color=color, **kwargs)

    texcoords: dict[str, TextureCoordinate] = property(lambda self: self._texcoords)


class NormalizedTexcoordImage(TexcoordImage):
    _normalization = "minmax"

    def __init__(self, *args, kind: NormalizationType = "minmax", **kwargs):
        self._normalization = kind
        super().__init__(*args, **kwargs)  # normalization is handled by TexcoordImage constructor


__all__ = ["TexcoordImage", "NormalizedTexcoordImage",
           "TextureCoordinate", "NormalizationType"]

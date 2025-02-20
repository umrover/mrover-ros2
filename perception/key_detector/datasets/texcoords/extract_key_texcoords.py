import glob
import pickle

from tqdm import tqdm
from multiprocessing import Pool

from keyrover.images.texcoord import *


reduce = "mean"


def get_key_texcoords(path: str) -> dict[str, TextureCoordinate]:
    img = TexcoordImage(path, reduce=reduce)
    return img.texcoords


if __name__ == "__main__":
    dataset = "v4-nodistort"

    paths = glob.glob(f"{dataset}/*.png")
    with Pool() as p:
        texcoords = list(tqdm(p.imap(get_key_texcoords, paths), total=len(paths)))

    with open(f"{dataset}/{reduce}_texcoords.bin", "wb") as file:
        pickle.dump(texcoords, file)

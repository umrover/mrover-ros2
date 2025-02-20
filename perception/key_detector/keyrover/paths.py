from typing import Final

import os


ROOT_PATH: Final[str] = f"{os.path.dirname(os.path.abspath(__file__))}/.."
DATASETS: Final[str] = f"{ROOT_PATH}/datasets"
MODELS_PATH: Final[str] = f"{ROOT_PATH}/models"

RAW_BACKGROUNDS: Final[str] = f"{DATASETS}/bg-20k"
RESIZED_BACKGROUNDS: Final[str] = f"{DATASETS}/bg-20k-resized"

RAW_DATASET: Final[str] = f"{DATASETS}/raw"
RAW_RENDERS: Final[str] = f"{RAW_DATASET}/renders"

YOLO_BINARY_DATASET: Final[str] = f"{DATASETS}/yolo"
YOLO_MULTI_DATASET: Final[str] = f"{DATASETS}/yolo-multiclass"

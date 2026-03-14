import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path
from time import time


# logging constants
_LOG_NAME = "esw"
_LOG_LEVEL = logging.INFO
_LOG_FMT = "%(asctime)s[%(levelname)-4s] %(message)s"
_LOG_DATE_FMT = "%m/%d/%Y %I:%M:%S%p "
_LOG_DIR = Path(Path.home(), f".{_LOG_NAME}", "logs")
_FORMATTER = logging.Formatter(fmt=_LOG_FMT, datefmt=_LOG_DATE_FMT)
_LOG_FILE = Path(_LOG_DIR, f"{_LOG_NAME}.{time()}.log")


def _configure_logging() -> logging.Logger:
    if not _LOG_DIR.exists():
        _LOG_DIR.mkdir(exist_ok=True, parents=True)

    console_handler = logging.StreamHandler()
    console_handler.setFormatter(_FORMATTER)
    file_handler = RotatingFileHandler(_LOG_FILE, maxBytes=5 * 1024 * 1024, backupCount=5)
    file_handler.setFormatter(_FORMATTER)

    # create logger
    logger = logging.getLogger(_LOG_NAME)
    logger.setLevel(_LOG_LEVEL)
    logger.addHandler(console_handler)
    logger.addHandler(file_handler)

    return logger


# global logging objects
esw_logger = _configure_logging()


def get_esw_root() -> Path:
    return Path(__file__).parents[2]

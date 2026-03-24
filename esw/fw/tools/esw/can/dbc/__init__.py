from functools import lru_cache
from pathlib import Path
from typing import cast

import cantools
from cantools.database import Database

from esw import esw_logger, get_esw_root


@lru_cache(maxsize=1)
def get_dbc(filepath: Path | None = None, dbc_name: str | None = None) -> Database:
    if filepath is None:
        if dbc_name is None:
            err = "one of filepath or dbc_name must be provided"
            esw_logger.error(err)
            raise RuntimeError(err)
        filepath = get_esw_root() / "dbc" / f"{dbc_name}.dbc"

    if not filepath.exists():
        esw_logger.error(f"DBC {filepath} does not exist")
        raise FileNotFoundError(filepath)

    try:
        db = cast(Database, cantools.database.load_file(filepath))
        return db
    except Exception as e:
        esw_logger.error(e)
        raise e

import shutil
from pathlib import Path

from esw import esw_logger
from esw.cubemx import run_cubemx_script


def _get_load_command(target_name):
    # MCUs usually start with "STM32", boards are "NUCLEO", "DISCO", "EVAL", etc.
    if target_name.upper().startswith("STM32"):
        return f"load {target_name}"
    else:
        return f"loadboard {target_name} nomode"


def generate_project(name: str, path: Path, mcu: str) -> None:
    load_cmd = _get_load_command(mcu)

    script = f"""
{load_cmd}
project name {name}
project path {path.absolute().resolve()}
project toolchain CMake
project compiler GCC
SetCopyLibrary copy as reference
project generate
exit
"""
    esw_logger.debug(script)

    if not run_cubemx_script(script):
        err = f"Failed to Generate Project {name}"
        esw_logger.error(err)
        raise RuntimeError(err)

    # force LibraryCopy=2
    ioc_path = (path / f"{name}.ioc").absolute().resolve()
    esw_logger.info(f"Patching IOC at {ioc_path}")
    with ioc_path.open("r", encoding="utf-8") as ioc_file:
        data = ioc_file.read()
    updated = data.replace("LibraryCopy=0", "LibraryCopy=2")
    with ioc_path.open("w", encoding="utf-8") as ioc_file:
        ioc_file.write(updated)
    drivers_dir = path / "Drivers"
    if drivers_dir.exists() and drivers_dir.is_dir():
        shutil.rmtree(drivers_dir)

    script = f"""
config load {str(ioc_path)}
project generate
exit
"""
    esw_logger.debug(script)

    if not run_cubemx_script(script):
        err = f"Failed to Re-Generate Project {name}"
        esw_logger.error(err)
        raise RuntimeError(err)

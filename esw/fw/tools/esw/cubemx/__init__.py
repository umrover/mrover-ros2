import subprocess
from functools import lru_cache
from pathlib import Path
from tempfile import NamedTemporaryFile
from typing import Union

from esw import esw_logger


@lru_cache(maxsize=1)
def get_installation():
    try:
        result = subprocess.run(
            ["which", "STM32CubeMX"],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except subprocess.CalledProcessError:
        err = "STM32CubeMX not found in PATH"
        esw_logger.error(err)
        raise RuntimeError(err)

    path_str = result.stdout.strip()
    if not path_str:
        err = "STM32CubeMX not found in PATH"
        esw_logger.error(err)
        raise RuntimeError(err)

    cubemx = Path(path_str).absolute().resolve()
    esw_logger.info(f"STM32CubeMX Found At: {cubemx}")
    return cubemx


def run_cubemx_script(script_content: str, return_output_on_failure: bool = False) -> Union[bool, tuple[bool, str]]:
    with NamedTemporaryFile(delete=True, mode="w+t") as temp_file:
        esw_logger.debug(f"Temporary file created: {temp_file.name}")
        temp_file.write(script_content)
        temp_file.flush()

        cmd = [str(get_installation()), "-q", str(temp_file.name)]
        esw_logger.debug(f"Command: {' '.join(cmd)}")
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, check=False, encoding="utf-8")
        except FileNotFoundError:
            error_msg = "CubeMX executable not found. Check installation path."
            esw_logger.error(error_msg)
            if return_output_on_failure:
                return False, error_msg
            return False

        if "KO" in result.stdout:
            output = result.stdout
            esw_logger.warning("CubeMX Script Command FAILED (indicated by 'KO' in stdout)")
            esw_logger.info(f"CubeMX Full Output:\n{output}")

            if return_output_on_failure:
                return False, output

        if result.returncode != 0:
            output = result.stderr or result.stdout
            esw_logger.warning(f"CubeMX Process FAILED (Return Code {result.returncode}):\n{output}")

            if return_output_on_failure:
                return False, output

            return False

        if return_output_on_failure:
            return True, ""

        return True

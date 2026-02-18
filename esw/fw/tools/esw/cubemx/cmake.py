import os
from datetime import datetime
from pathlib import Path
from typing import Any

from jinja2 import Environment, FileSystemLoader

from esw import esw_logger


_C_STANDARD: int = 17
_CXX_STANDARD: int = 23


def _clean_project(path: Path) -> None:
    esw_logger.info(f"Cleaning Existing CMake Project {path.absolute().resolve()}")
    clangd = path / ".clangd"
    cmakelists = path / "CMakeLists.txt"
    cmakepresets = path / "CMakePresets.json"
    cmakelists_stm = path / "cmake" / "stm32cubemx" / "CMakeLists.txt"

    if clangd.exists() and clangd.is_file():
        clangd.unlink()
    if cmakelists.exists() and cmakelists.is_file():
        cmakelists.unlink()
    if cmakepresets.exists() and cmakepresets.is_file():
        cmakepresets.unlink()
    if cmakelists_stm.exists() and cmakelists_stm.is_file():
        cmakelists_stm.unlink()
    if cmakelists_stm.parent.exists() and cmakelists_stm.parent.is_dir():
        cmakelists_stm.parent.rmdir()


def _get_cmakelists_context(name: str, path: Path, root: Path, libs: list[str]) -> dict[str, Any]:
    # find source file directory
    src_dir: Path | None = None
    possible_srcs = [path / "Src", path / "Core" / "Src"]
    for possible_src in possible_srcs:
        if possible_src.exists() and possible_src.is_dir():
            src_dir = possible_src.relative_to(path)
            esw_logger.info(f"Found Source Directory {src_dir.absolute().resolve()}")
    if not src_dir:
        err = f"Could not find Source Directory under {path.absolute().resolve()}"
        esw_logger.error(err)
        raise RuntimeError(err)

    # find header file directory
    inc_dir: Path | None = None
    possible_incs = [path / "Inc", path / "Core" / "Inc"]
    for possible_inc in possible_incs:
        if possible_inc.exists() and possible_inc.is_dir():
            inc_dir = possible_inc.relative_to(path)
            esw_logger.info(f"Found Header Directory {inc_dir.absolute().resolve()}")
    if not inc_dir:
        err = f"Could not find Header Directory under {path.absolute().resolve()}"
        esw_logger.error(err)
        raise RuntimeError(err)

    # find driver script
    scripts = [p for p in path.iterdir() if p.is_file() and p.suffix == ".s"]
    if len(scripts) == 0:
        err = f"No .s script found in directory {path.absolute().resolve()}"
        esw_logger.error(err)
        raise FileNotFoundError(err)
    if len(scripts) > 1:
        err = f"Multiple .s scripts found: {[p.name for p in scripts]}"
        esw_logger.error(err)
        raise RuntimeError(err)
    driver_script = scripts[0].relative_to(path)
    if driver_script:
        esw_logger.info(f"Found Driver Script {driver_script.absolute().resolve()}")
    else:
        err = f"Could not find Driver Script under {path.absolute().resolve()}"
        esw_logger.error(err)
        raise RuntimeError(err)

    # find relative fwlib path
    cmake_path = path.absolute().resolve()
    lib_path = (root / "lib").absolute().resolve()
    lib_relative_path = os.path.relpath(lib_path, cmake_path)
    esw_logger.info(f"Found fwlib at {lib_relative_path}")

    return {
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "c_standard": _C_STANDARD,
        "cxx_standard": _CXX_STANDARD,
        "project_name": name,
        "mx_src_dir": src_dir,
        "mx_inc_dir": inc_dir,
        "mx_startup_s": driver_script,
        "lib_relative_path": lib_relative_path,
        "libs": libs,
    }


def _get_clangd_context() -> dict[str, Any]:
    NUM_PATHS = 3  # gcc include directory, c++ include directory, c++ eabi include directory
    opt_st = Path("/opt/st")
    if not opt_st.exists():
        return {
            "includes": None,
            "c_standard": _C_STANDARD,
            "cxx_standard": _CXX_STANDARD,
        }

    base_dirs = sorted(opt_st.glob("stm32cubeclt_*/GNU-tools-for-STM32/arm-none-eabi"), reverse=True)
    if not base_dirs:
        return {
            "includes": None,
            "c_standard": _C_STANDARD,
            "cxx_standard": _CXX_STANDARD,
        }

    base_dir = base_dirs[0]
    paths: list[Path] = []
    c_include = base_dir / "include"
    if c_include.exists():
        paths.append(c_include)

    cpp_root = base_dir / "include" / "c++"
    if cpp_root.exists():
        version_dirs = sorted([d for d in cpp_root.iterdir() if d.is_dir() and d.name != "arm-none-eabi"], reverse=True)
        if version_dirs:
            latest_cpp_version = version_dirs[0]
            paths.append(latest_cpp_version)
            target_path = latest_cpp_version / "arm-none-eabi"
            if target_path.exists():
                paths.append(target_path)

    return {
        "includes": paths if paths and len(paths) == NUM_PATHS else None,
        "c_standard": _C_STANDARD,
        "cxx_standard": _CXX_STANDARD,
    }


def configure_cmake(name: str, path: Path, root: Path, ctx: Path, libs: list[str]) -> None:
    _clean_project(path)

    env = Environment(loader=FileSystemLoader(ctx))

    cmake_context = _get_cmakelists_context(name, path, root, libs)
    cmake_template = env.get_template("templates/CMakeLists.txt.j2")
    cmakelists = path / "CMakeLists.txt"
    esw_logger.info(f"Writing CMakeLists.txt to {cmakelists.absolute().resolve()}")
    with cmakelists.open("w") as handle:
        handle.write(cmake_template.render(cmake_context))

    cmake_presets_template = env.get_template("templates/CMakePresets.json.j2")
    cmakepresets = path / "CMakePresets.json"
    esw_logger.info(f"Writing CMakePresets.json to {cmakepresets.absolute().resolve()}")
    with cmakepresets.open("w") as handle:
        handle.write(cmake_presets_template.render())

    clangd_context = _get_clangd_context()
    if clangd_context is not None:
        if clangd_context.get("includes") is None:
            esw_logger.warning(
                "Could not validate GCC installation, there is likely something wrong with the STM32CubeCLT installation"
            )
            clangd_context["includes"] = []
    clangd_template = env.get_template("templates/.clangd.j2")
    clangd = path / ".clangd"
    esw_logger.info(f"Writing .clangd to {clangd.absolute().resolve()}")
    with clangd.open("w") as handle:
        handle.write(clangd_template.render(clangd_context))


def configure_clang(path: Path, ctx: Path) -> None:
    clangd = path / ".clangd"
    if clangd.exists():
        return

    env = Environment(loader=FileSystemLoader(ctx))
    clangd_context = _get_clangd_context()
    if clangd_context is not None:
        if clangd_context.get("includes") is None:
            esw_logger.warning(
                "Could not validate GCC installation, there is likely something wrong with the STM32CubeCLT installation"
            )
            clangd_context["includes"] = []
    clangd_template = env.get_template("templates/.clangd.j2")
    esw_logger.info(f"Writing .clangd to {clangd.absolute().resolve()}")
    with clangd.open("w") as handle:
        handle.write(clangd_template.render(clangd_context))

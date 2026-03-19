# Build Tools

This document covers the build tooling and other utilities developed to aid the
development process.

## Scripts

All of the scripts found in `scripts/` serve as wrappers of either python tools
or vendor utilities (e.g. MJBots, STMicroelectronics), and should be used as
simple interfaces to these tools.

### `new.sh`

The `scripts/new.sh` script is designed to aid the creation of new STM32 projects
that leverage the HAL libraries provided by STM. The script can be run as follows.

```bash
./scripts/new.sh --mcu <mcu> --src <path-to-new-project> [--lib <library>]
# OR
./scripts/new.sh --board <board> --src <path-to-new-project> [--lib <library>]
# OR
./scripts/new.sh --help  # to display the options menu
```

In the invocation above, the script accepts either an MCU (e.g. `STMG431CBTx`)
or a development board (e.g. `NUCLEO-G431RB`). 

!!! note
    Note that under the hood, this script emplaces this parameter directly into 
    a headless STM32CubeMX script, which will fail if the MCU or board provided
    is invalid. Use the STM32CubeMX project creation wizard to see possible MCU
    options available, and to verify the correct part information.

Under the hood, this script will run CubeMX and create an STM32 project for
the specified MCU. It does this by running a script of STM32CubeMX commands
that create the project in the correct CMake/GCC configuration, and then
manually patches the IOC to ensure some settings that aren't linked to
STM32CubeMX commands are correctly set.

The directory provided by the `--src` flag will be used for the creation of the
new project, and once the script finishes execution the project should be able
to be built.

Libraries can be provided with the `--lib` flag; if, for example, the new project
should link with ESW's library for commonly used stm32 header files (defined by
`lib/stm32/CMakeLists.txt`) and the CAN messages defined by the auto-generated
DBC headers (defined by `lib/dbc/CMakeLists.txt`), the script should be run with
`... --lib stm32 --lib dbc` options. This will result in the created
`<src>/CMakeLists.txt` linking against these libraries. This is not necessary to
get all the libraries correct at this point, as the file can be modified later.

The created `<src>/CMakeLists.txt` will then contain the following section.
```cmake
# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
    stm32
    dbc
)
```

!!! note
    Future calls to the automated python tools that manage the auto-generated CMake
    files will overwrite this section, so these parameters will need to be provided
    to these scripts as well.

### `build.sh`

The build script is a wrapper around the CMake and the GCC distribution bundled with
the STM32CubeCLT, as well as the STMCubeProgrammer CLI. The build script can be run
in the following configurations.

1\. Clean Project

The script's `--clean` flag will remove all build artifacts from the project source.
This may be necessary to run after some updates to build files, as CMake is only
configured once and the cached build artifacts are used after.

```bash
./scripts/build.sh --src <src> --clean
```

2\. Build Project

The default behavior of the script is to build the specified project using CMake.
The script consumes the `--src <path-to-project>` flag to denote the directory
containing the project. If the project target is different from the directory name
(which is not the case for projects generated with the `./scripts/new.sh` script)
the `--target <target>` is provided. Additionally, the build preset can be altered
with the `--preset <preset>` flag. The default preset provided with the CMake
configuration is `Debug`, but `Release` is also valid. If no preset is specified,
the script will default to `Debug`. To build a project, the script can be run as
follows.

```bash
./scripts/build.sh --src <path-to-project> [--preset <preset>] [--target <target>]
```

3\. Flash Project

To flash the executable file for a project to an MCU, use the `--flash` flag. When
set, the script will attempt a build as above, and if successful invoke the
STM32CubeProgrammer CLI to connect to an ST-LINK and flash the executable via SWD.

!!! important
    Presently the script will only support flashing STM32 MCUs with SWD, and not JTAG.

```bash
./scripts/build.sh --src <path-to-project> [--preset <preset>] --flash
```

Finally, the build script will ensure the existance of a `<src>/.clangd` file for
development environment compatability. This needs to be generated per-project as
it contains some project-specific and system-specific parameters.

### `style.sh`

The style script enforces code style across all ESW code. It runs in CI, and must
be passing for PRs to be accepted. The script can be run with the following arguments.

```bash
./scripts/style.sh [--format] [--lint] [--fix] [--verbose]
```

If no options are provided, the `--format` run configuration will be used by default.
The functionality of the options is enumerated below.

- `--format`

This configuration will run `clang-format` and `ruff` to format `C/C++` and `Python`
code, respectively.

- `--lint`

This configuration will run linters `ruff` and `shellcheck` for `Python` and `shell`
scripts, respectively. This configuration will also run `ty` for static type analysis
of `Python` scripts.

- `--fix`

This configuration will fix all possible formatting and linting issues found with the
associated flags the script is run with. If the script finishes unsuccessfully with
this flag, there are issues with the codebase that the tools cannot automatically
resolve.

- `--verbose`

This flag runs the script with verbose output.

### `fdcanusb.sh`

This script is a wrapper over MJBot's `fdcanusb_daemon`. It is designed to run with
the MJBots FDCANUSB to allow connectivity to CAN networks over USB. The script is run
as follows.

```bash
sudo ./scripts/fdcanusb.sh --net <vcan-network>
```

!!! important
    This script must be run as `sudo` on linux systems to allow the virtual CAN
    network interface to be created and started.

The `--net` flag specifies the virtual CAN network the fdcanusb should use. To work
with the ROS2 stack, this should be something like `can[0-3]`. The script runs the
underlying `fdcanusb_daemon` in verbose mode, so the full CAN frame of all messages
on the bus will be directed to standard output.

!!! TODO
    MJBots no longer supports the `fdcanusb_daemon` (this script currently pulls the source
    for this from an archive). This script should be updated to not depend on this binary.

## Python Tools

The ESW python tools (located in `tools/`) are designed to enable rapid development, testing, 
and validation of the embedded hardware and software stack. Currently, the following submodules of 
the main `esw` python module are stable. Other submodules (e.g. `esw.bmc`) are product-specific,
and not documented here.

1\. `esw.stlink`

This module contains scripts to handle serial data from the VCP-TX and VCP-RX pins on the ST-LINKv3.
This enables serial data to be sent and received from the MCU via UART. The `tools/scripts/monitor.py`
script (aliased with the `monitor` just recipe) will use this stack to display data sent from the MCU
with the logger, provided to the firmware via the `util` CMake package.

2\. `esw.cubemx`

This module contains scripts to handle the creation and management of STM32 CMake projects.
The `tools/scripts/generate_project.py` script is uesd by the `scripts/new.sh` script, as is
the `tools/scripts/update_cmake_cfg.py` script, which re-creates all CMake files for a project.

3\. `esw.can`

This module contains the functionality to use both the DBC files as well as the FDCANUSB to send
and receive can messages. Many scripts in `tools/scripts` rely on this functionality to send and
receive CAN messages to the MCUs. The `tools/scripts/can_header_gen.py` script is used by the
`lib/dbc/CMakeLists.txt` file to generate DBC header files for the `dbc` library.

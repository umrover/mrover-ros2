# MRover ROS 2: Portability and Jazzy Migration Plan

## Goal

- Make the dev build portable across Unix systems: macOS and any Linux distro.
- Upgrade ROS 2 Humble to Jazzy.
- One-command setup. No environment drift between developers.

## Past failures

- No pinned environment. Builds used ad hoc local state. Hardcoded paths got
  committed (e.g. `libpython3.11.dylib` in `cmake/macros.cmake`).
- No macOS CI. Working setups broke within weeks.
- macOS gets re-attempted at the start of each school year, then abandoned
  soon after. No progress carries over.
- Scoped as porting the whole repo. Contributors hit CUDA and CAN code that
  cannot build on any Mac.

Fixed here: pinned lockfile, macOS CI, work on `main`, explicit scope.

## Architecture: two tiers

Perception, ESW, CAN, and firmware only run on the rover. Everything else is
portable.

### Tier 1: rover (full build)

- Targets: Jetson and Ubuntu full-stack machines.
- OS: Ubuntu (Noble for Jazzy).
- ROS: native apt.
- Setup: Ansible.
- Builds everything, including CUDA, ZED, CAN, ESW.
- Must build natively. Perception links L4T CUDA, TensorRT, and ZED. conda
  cannot supply those.

### Tier 2: dev (portable build)

- Targets: macOS, any Linux, WSL2, CI.
- ROS: RoboStack `ros-jazzy-*` conda packages.
- Setup: pixi with a committed lockfile.
- Builds the portable subset only.
- Ubuntu devs on the portable subset use Tier 2 too. This avoids two
  dependency systems and version drift.

## Supported platforms

Support levels:

- Supported: covered by CI, merges gated, breakage blocks release.
- Best-effort: expected to work, no CI, fixed when reported.
- Unsupported: not a target.

| Device / OS                    | Arch    | Tier | Support     | Notes                                               |
| ------------------------------ | ------- | ---- | ----------- | --------------------------------------------------- |
| Jetson AGX Orin                | aarch64 | 1    | Supported   | Ubuntu via JetPack. Validated on-device, not in CI. |
| Ubuntu workstation, full stack | x86_64  | 1    | Supported   | Ubuntu 24.04 Noble. Native apt and CUDA.            |
| macOS, Apple Silicon           | arm64   | 2    | Supported   | macOS 14 or newer. Primary macOS target.            |
| macOS, Intel                   | x86_64  | 2    | Unsupported | Use Docker or a Linux machine.                      |
| Linux, glibc, any distro       | x86_64  | 2    | Supported   | Distro-agnostic.                                    |
| Linux, glibc, any distro       | aarch64 | 2    | Best-effort | ARM laptops and SBCs. No dedicated CI.              |
| WSL2 (Ubuntu under Windows)    | x86_64  | 2    | Supported   | Treated as Linux.                                   |
| Linux, musl (Alpine)           | any     | n/a  | Unsupported | conda-forge has no musl builds.                     |
| Native Windows (non-WSL)       | x86_64  | n/a  | Unsupported | Use WSL2.                                           |
| NixOS                          | x86_64  | 2    | Best-effort | Non-FHS layout conflicts with conda.                |

### macOS

- Supported: Apple Silicon (`osx-arm64`), macOS 14 or newer.
- Not supported: Intel (`osx-64`). Intel Macs are aging out and macOS 26
  dropped Intel. Supporting it doubles the CI matrix for a shrinking user base.
- Intel Mac users: use Docker or a Linux machine.

### Linux

- Any glibc-based distro works. conda-forge binaries are distro-agnostic.
- Families: Debian and Ubuntu, Fedora and RHEL, Arch, openSUSE.
- Not supported: musl distros (Alpine). conda-forge has no musl builds.
- Best-effort: NixOS.
- Tier 1 still requires Ubuntu (apt and CUDA path).

### WSL2

- Supported. WSL2 is a real Linux kernel. The pixi build treats it as Linux.
- Recommended path for Windows users. Native Windows is not supported.
- The web GUI works via localhost in a Windows browser.
- The simulator GPU path (WebGPU over WSLg) is the least proven part.
- Multi-machine DDS needs WSL2 mirrored networking mode.
- WSL1 is not supported.

## Principles

- One ROS distro fleet-wide (Jazzy). Packaging may differ (apt vs RoboStack).
  The distro may not.
- RoboStack-Jazzy and apt-Jazzy interoperate over DDS. Same distro, same type
  hashes.
- Never mix ROS distros across the fleet.
- One RMW everywhere: the Jazzy default (Fast DDS).
- Build from source per target. Never ship binaries between platforms.
- The lockfile is committed. Setup is `pixi install`.
- The portable scope is a CMake profile, not an accident.

## Scope: portable vs rover-only

| Component                                                                                                                       | Tier                   | Notes                            |
| ------------------------------------------------------------------------------------------------------------------------------- | ---------------------- | -------------------------------- |
| Basestation GUI (FastAPI, Bun/Vue)                                                                                              | Portable               | Web and Python                   |
| Autonomy Python (navigation, state_machine, localization, superstructure)                                                       | Portable               | Pure Python and rclpy            |
| `lie` / manifpy                                                                                                                 | Portable               | manif from conda-forge or source |
| Simulator (Dawn, GLFW, Bullet, imgui, Assimp)                                                                                   | Portable               | Highest-effort portable item     |
| C++ nodes: tag_detector, cost_map, arm_controller, differential_drive_controller, pose_filter, heading_filter, rover_gps_driver | Portable               | OpenCV and Eigen only            |
| camera_client (Qt5, GStreamer)                                                                                                  | Portable, low priority | macOS codec coverage is patchier |
| Message, service, action generation                                                                                             | Portable               | Pure ROS                         |
| ZED wrapper, object_detector, tensorrt                                                                                          | Rover only             | Needs CUDA and ZED SDK           |
| can_bridge                                                                                                                      | Rover only             | Needs netlink/SocketCAN          |
| ESW hardware bridges, u2d2_bridge                                                                                               | Rover only             | Needs CAN and `esw/fw`           |
| gst_camera_server                                                                                                               | Rover only             | Needs libudev                    |
| `esw/fw` firmware                                                                                                               | Rover only             | Bare-metal STM32                 |

The CMake already gates most non-portable targets. Phase 1 makes that explicit.

## Dependencies (Tier 2)

Resolve in this order. Build-from-source is a last resort.

1. RoboStack `robostack-jazzy`: ROS packages.
2. conda-forge: system libs (glfw, bullet, assimp, eigen, opencv, qt,
   gstreamer, compilers, cmake, ninja).
3. PyPI via pixi `[pypi-dependencies]`: pip-only packages (moteus, pyubx2,
   pymap3d). Wheels, not source.
4. Vendored `deps/` submodules: built by colcon.
5. Build from source: only Dawn (`deps/dawn`), and manif if not on conda-forge.

- Build Dawn once per platform in CI. Host it as a release artifact. Per-dev
  source builds are the fallback.
- Put conda-available packages (numpy, opencv) in `[dependencies]`. Keep
  pip-only ones in `[pypi-dependencies]`. This reduces pixi solve conflicts.

## Tooling: pixi

pixi provides:

- Unified conda and PyPI management, both in one lockfile.
- A committed lockfile (`pixi.lock`) per platform. Deterministic builds.
- One manifest (`pixi.toml`) with targets `linux-64`, `linux-aarch64`,
  `osx-arm64`.
- Multiple environments (basestation, sim, dev, test) in one manifest.
- A task runner for build, launch, and tooling commands.
- Project-local. No conda or pip install needed. No global activation hook.

## Phase 0: prerequisites

- [x] `robostack-jazzy` confirmed GA for `osx-arm64`. Proceeding directly to
      Jazzy. The `ros-humble-ros-base` osx-arm64 package requires Python 3.9,
      not 3.10, making a clean Humble baseline impossible on macOS.
- [ ] Confirm JetPack 7 supports the AGX Orin and ships Ubuntu 24.04. If the
      Orin stays on JetPack 6, the rover stays on Humble until JetPack 7.
- [x] conda-forge `osx-arm64` builds confirmed for: glfw, bullet, assimp,
      eigen, opencv, qt, gstreamer, manif.

The rover (Tier 1) upgrades to Ubuntu 24.04 Noble and Jazzy alongside Tier 2.
One distro fleet-wide. Rover upgrade is gated on JetPack 7 availability for
the Jetson AGX Orin.

## Phases

### Phase 1: define the portable subset

- [ ] Add a CMake profile (`MROVER_PORTABLE`) that excludes perception, CAN,
      ESW, and firmware.
- [ ] Audit `find_package` and `NOT APPLE` gating for holes.

### Phase 2: pixi on Jazzy

- [ ] Write `pixi.toml`: `robostack-jazzy`, Python 3.12, three platforms.
- [ ] Commit `pixi.lock`.
- [ ] Run the basestation GUI and autonomy Python on macOS.
- [ ] Remove hardcoded paths (see Technical debt).

### Phase 3: macOS CI

- [ ] Add a macOS CI job: `pixi install`, build the portable subset.
- [ ] Add a portable-Linux CI job (pixi, not the apt container).
- [ ] Make both required for merge.

### Phase 4: C++ portable nodes

- [ ] Build tag_detector, cost_map, controllers, and filters under pixi on
      macOS.
- [ ] Fix code-level portability issues.

### Phase 5: simulator

- [ ] Build Dawn on macOS (Metal backend).
- [ ] Host CI-built Dawn artifacts for macOS.
- [ ] Align the imgui wgpu backend with the Dawn `webgpu.h` version.
- [ ] Run the simulator on macOS.

### Phase 6: Tier 1 rover upgrade to Jazzy

- [ ] Upgrade Tier 1 Ansible to Ubuntu 24.04 Noble and `ros-jazzy-*` apt packages.
- [ ] Verify DDS message compatibility between Tier 1 and Tier 2.
- [ ] Upgrade the Jetson last, after JetPack 7 is available for the AGX Orin.

## Technical debt to fix

- `cmake/macros.cmake`: remove the hardcoded `libpython*.dylib` link lines.
- `environment.bash`: remove the hardcoded `/home/quintin/ros2_ws` path.
- `build.sh`: guard the CUDA exports so they skip non-CUDA platforms.
- `CMakeLists.txt`: replace `find_package(PythonLibs)` with
  `find_package(Python3)`.

## Risks

| Risk                                  | Mitigation                                                    |
| ------------------------------------- | ------------------------------------------------------------- |
| JetPack 7 slips or omits Orin support | Rover stays on Humble until JetPack 7. Tier 2 proceeds on Jazzy; fleets must not talk until rover upgrades. |
| RoboStack package gaps                | Work around or pin via lockfile. Keep exotic packages Tier 1. |
| Dawn or imgui version skew            | Pin the Dawn SHA. Host prebuilt artifacts via GitHub releases. |
| Work decays again                     | macOS CI required for merge. No long-lived branch.            |

## Anti-patterns

- No long-lived feature branch. Merge each working slice to `main`.
- No hardcoded machine-specific paths.
- No hybrid apt-on-Ubuntu-dev plus conda-elsewhere.
- No building perception, CAN, or ESW on macOS.
- No mixed ROS distros across the fleet.

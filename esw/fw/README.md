# mrover-esw

This repository contains all relevant information for the Embedded Software (ESW) MRover subteam.

## Repo Structure

Documentation can be found in `docs/` and can be viewed [here](https://umrover.github.io/mrover-esw/).
Instructions for developing and building the documentation locally can be found in the `docs/README.md` file.

The `src/` directory contains the source code for the ESW subteam. As of now, only firmware should be placed here.
All ROS2 code should go in the [mrover-ros2 repository](https://github.com/umrover/mrover-ros2).

The `lib/` directory contains library code including hardware drivers and utility code.

The `starter-projects/` directory contains the starter code for the new member starter projects.
Information about how to start the starter projects can be found in the documentation.

The `scripts/` directory contains various build and utility scripts.

The `tools/` directory contains our Python utilities. This includes scripts for generating new projects
and helpful CAN utlities.

The `dbc/` directory contains our CAN database files.

The `ci.json` file contains the paths for the continuous integration (CI) system to build and test the code.
We use GitHub Actions for our CI system and the configuration can be found in `.github/workflows/`.

The `justfile` contains various helpful commands for using the repository. You can get started with using
`just` by checking out the [GitHub repository](https://github.com/casey/just).

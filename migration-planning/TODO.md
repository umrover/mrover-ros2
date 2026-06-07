# Migration TODOs

## CI environment drift

The GitHub Actions `ci.yml` build job uses the `umrover1/ros2:latest` Docker image, which
ships ROS 2 Humble and sources `/opt/ros/humble/setup.sh`. Local dev now targets Jazzy via
pixi/RoboStack. These are out of sync.

Action: update CI to use a Jazzy-based image (or a pixi-based setup) so the CI build
environment matches the local dev environment.

Related: `build.sh` already branches on `PIXI_PROJECT_ROOT` to handle both paths, but the
Docker image does not run pixi, so CI always takes the non-pixi (Humble) path.

#pragma once

#include <string>
#include <vector>

#include <mujoco/mujoco.h>

namespace mrover::sim {

    // A map object whose driving joints should receive a torque actuator. links are
    // URDF link names (e.g. "arm_a_link"); the actuator drives the joint that moves
    // that link and is named "<object>/<link>" so the node can look it up.
    struct ActuatedObject {
        std::string object;
        std::vector<std::string> links;
    };

    // Assembles a MuJoCo world from a map YAML (config/simulator/<map>.yaml): each
    // entry is a committed MJCF model (simulator/models/mjcf/*.xml) placed at a
    // position+orientation. fixed objects are welded; others get a free joint.
    //
    // For every ActuatedObject, a force-controlled motor is added to each named
    // joint so the physics node can command throttle/velocity/position.
    //
    // Returns a compiled model (caller owns it; free with mj_deleteModel) or throws.
    mjModel* buildSceneFromMap(std::string const& mapFile,
                               std::vector<ActuatedObject> const& actuated = {});

} // namespace mrover::sim

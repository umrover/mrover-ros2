// Phase 0 spike: verify the prebuilt MuJoCo C library links and simulates.
// Loads a trivial scene (a box falling onto a plane), steps it, and prints the
// resting height. Not part of the ROS build; compiled standalone for validation.

#include <cstdio>
#include <mujoco/mujoco.h>

namespace {
    constexpr char const* kSceneXml = R"(
<mujoco>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <geom type="plane" size="5 5 0.1"/>
    <body name="box" pos="0 0 1">
      <freejoint/>
      <geom type="box" size="0.1 0.1 0.1" mass="1"/>
    </body>
  </worldbody>
</mujoco>
)";
} // namespace

int main() {
    char const* path = "/tmp/mj_spike_scene.xml";
    if (FILE* f = std::fopen(path, "w")) {
        std::fputs(kSceneXml, f);
        std::fclose(f);
    } else {
        std::fprintf(stderr, "could not write scene file\n");
        return 1;
    }

    char error[1000] = "";
    mjModel* model = mj_loadXML(path, nullptr, error, sizeof(error));
    if (!model) {
        std::fprintf(stderr, "mj_loadXML failed: %s\n", error);
        return 1;
    }

    mjData* data = mj_makeData(model);
    std::printf("MuJoCo %s loaded: nq=%ld nv=%ld nbody=%ld\n", mj_versionString(),
                static_cast<long>(model->nq), static_cast<long>(model->nv), static_cast<long>(model->nbody));

    for (int i = 0; i < 1000; ++i) mj_step(model, data);

    int box = mj_name2id(model, mjOBJ_BODY, "box");
    std::printf("after %.3f s, box z = %.4f (expect ~0.1)\n", data->time, data->xpos[3 * box + 2]);

    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
}

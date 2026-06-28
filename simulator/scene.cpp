#include "scene.hpp"

#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace mrover::sim {

    namespace {

        namespace fs = std::filesystem;

        // simulator/models, holding mjcf/*.xml plus meshes/ + textures/ (all committed via LFS).
        fs::path modelsDir() {
            try {
                fs::path share = ament_index_cpp::get_package_share_directory("mrover");
                if (fs::path p = share / "models"; fs::exists(p)) return p;
            } catch (...) {
            }
            if (fs::path p = fs::path{"simulator"} / "models"; fs::exists(p)) return p;
            throw std::runtime_error("could not locate simulator/models");
        }

        fs::path resolveMapPath(std::string const& mapFile) {
            if (fs::exists(mapFile)) return mapFile;
            try {
                fs::path share = ament_index_cpp::get_package_share_directory("mrover");
                if (fs::path p = share / "config" / "simulator" / mapFile; fs::exists(p)) return p;
            } catch (...) {
            }
            if (fs::path p = fs::path{"config"} / "simulator" / mapFile; fs::exists(p)) return p;
            throw std::runtime_error("map file not found: " + mapFile);
        }

        void set3(float (&dst)[3], float a, float b, float c) {
            dst[0] = a;
            dst[1] = b;
            dst[2] = c;
        }
        void set3(double (&dst)[3], double a, double b, double c) {
            dst[0] = a;
            dst[1] = b;
            dst[2] = c;
        }

        // The world spec is assembled programmatically, so it has no lights, sky, or
        // materials by default; without these the viewer shows flat, near-black geometry.
        void addVisuals(mjSpec* world, mjsBody* worldbody) {
            // Brighten the headlight so untextured collision geometry reads as 3D even
            // before any scene lights are considered.
            set3(world->visual.headlight.ambient, 0.4F, 0.4F, 0.4F);
            set3(world->visual.headlight.diffuse, 0.5F, 0.5F, 0.5F);
            set3(world->visual.headlight.specular, 0.2F, 0.2F, 0.2F);
            world->visual.headlight.active = 1;
            world->visual.quality.shadowsize = 4096;

            // Sun: a directional light that casts shadows for depth cues.
            mjsLight* sun = mjs_addLight(worldbody, nullptr);
            sun->type = mjLIGHT_DIRECTIONAL;
            sun->castshadow = 1;
            set3(sun->pos, 0.0, 0.0, 10.0);
            set3(sun->dir, 0.3, 0.2, -1.0);
            set3(sun->ambient, 0.2F, 0.2F, 0.2F);
            set3(sun->diffuse, 0.8F, 0.8F, 0.8F);
            set3(sun->specular, 0.3F, 0.3F, 0.3F);

            // Gradient skybox (cube texture used automatically as the background).
            mjsTexture* sky = mjs_addTexture(world);
            sky->type = mjTEXTURE_SKYBOX;
            sky->builtin = mjBUILTIN_GRADIENT;
            set3(sky->rgb1, 0.45, 0.6, 0.8);
            set3(sky->rgb2, 0.1, 0.12, 0.16);
            sky->width = 512;
            sky->height = 512;
            mjs_setName(sky->element, "skybox");

            // Checkerboard ground texture + material so the plane has scale and texture.
            mjsTexture* grid = mjs_addTexture(world);
            grid->type = mjTEXTURE_2D;
            grid->builtin = mjBUILTIN_CHECKER;
            set3(grid->rgb1, 0.28, 0.24, 0.2);
            set3(grid->rgb2, 0.38, 0.33, 0.27);
            grid->mark = mjMARK_EDGE;
            set3(grid->markrgb, 0.5, 0.5, 0.5);
            grid->width = 512;
            grid->height = 512;
            mjs_setName(grid->element, "grid");

            mjsMaterial* groundMat = mjs_addMaterial(world, nullptr);
            mjs_setName(groundMat->element, "groundmat");
            mjs_setInStringVec(groundMat->textures, mjTEXROLE_RGB, "grid");
            groundMat->texrepeat[0] = 12.0F;
            groundMat->texrepeat[1] = 12.0F;
            groundMat->texuniform = 1;
            groundMat->reflectance = 0.1F;
        }

        void addGroundPlane(mjsBody* worldbody) {
            mjsGeom* ground = mjs_addGeom(worldbody, nullptr);
            ground->type = mjGEOM_PLANE;
            ground->size[0] = 50.0;
            ground->size[1] = 50.0;
            ground->size[2] = 0.1;
            mjs_setString(ground->material, "groundmat");
            mjs_setName(ground->element, "ground");
        }

        // Add a force (torque) motor to the joint that drives each requested link.
        // The node implements its own PD on top, so a plain motor (fixed unit gain,
        // no bias) is all we need here.
        void addActuators(mjSpec* world, std::vector<ActuatedObject> const& actuated) {
            for (ActuatedObject const& obj: actuated) {
                for (std::string const& link: obj.links) {
                    std::string bodyName = obj.object + "/" + link;
                    mjsBody* body = mjs_findBody(world, bodyName.c_str());
                    if (!body) {
                        std::fprintf(stderr, "[scene] no body '%s' to actuate\n", bodyName.c_str());
                        continue;
                    }
                    mjsElement* jointEl = mjs_firstChild(body, mjOBJ_JOINT, /*recurse=*/0);
                    if (!jointEl) {
                        std::fprintf(stderr, "[scene] body '%s' has no joint to actuate\n", bodyName.c_str());
                        continue;
                    }
                    // mjString is the library's std::string; read it through the C
                    // accessor so we never touch it across the (possibly mismatched)
                    // libstdc++ ABI boundary.
                    char const* jointName = mjs_getString(mjs_getName(jointEl));
                    if (!jointName) {
                        std::fprintf(stderr, "[scene] joint of '%s' has no name\n", bodyName.c_str());
                        continue;
                    }

                    mjsActuator* act = mjs_addActuator(world, nullptr);
                    act->trntype = mjTRN_JOINT;
                    mjs_setString(act->target, jointName);
                    act->gaintype = mjGAIN_FIXED;
                    act->gainprm[0] = 1.0;
                    act->biastype = mjBIAS_NONE;
                    act->gear[0] = 1.0;
                    act->forcelimited = mjLIMITED_TRUE;
                    act->forcerange[0] = -200.0;
                    act->forcerange[1] = 200.0;
                    mjs_setName(act->element, bodyName.c_str());
                }
            }
        }

    } // namespace

    mjModel* buildSceneFromMap(std::string const& mapFile, std::vector<ActuatedObject> const& actuated) {
        fs::path mapPath = resolveMapPath(mapFile);
        YAML::Node config = YAML::LoadFile(mapPath.string());
        YAML::Node objects = config["objects"];
        if (!objects) throw std::runtime_error("'objects' not defined in " + mapPath.string());

        fs::path models = modelsDir();

        mjSpec* world = mj_makeSpec();
        // One meshdir resolves every attached model's meshes at compile time.
        mjs_setString(world->compiler.meshdir, (models / "meshes").string().c_str());

        mjsBody* worldbody = mjs_findBody(world, "world");
        addVisuals(world, worldbody);
        addGroundPlane(worldbody);

        // Child specs must outlive the compile.
        std::vector<mjSpec*> children;

        for (auto const& entry: objects) {
            auto name = entry.first.as<std::string>();
            YAML::Node node = objects[name];
            auto uri = node["uri"].as<std::string>();
            bool fixed = node["fixed"] && node["fixed"].as<bool>();
            auto position = node["position"] ? node["position"].as<std::vector<double>>() : std::vector<double>{0, 0, 0};
            auto orientation = node["orientation"] ? node["orientation"].as<std::vector<double>>() : std::vector<double>{0, 0, 0, 1};
            if (position.size() != 3) throw std::invalid_argument{name + ": position must have 3 elements"};
            if (orientation.size() != 4) throw std::invalid_argument{name + ": orientation must have 4 elements"};

            fs::path modelPath = models / uri;
            if (!fs::exists(modelPath)) {
                std::fprintf(stderr, "[scene] skipping %s: model not found at %s\n", name.c_str(), modelPath.c_str());
                continue;
            }

            char error[1024] = "";
            mjSpec* child = mj_parseXML(modelPath.c_str(), nullptr, error, sizeof(error));
            if (!child) {
                std::fprintf(stderr, "[scene] skipping %s: %s\n", name.c_str(), error);
                continue;
            }
            children.push_back(child);

            mjsBody* childWorld = mjs_findBody(child, "world");
            mjsBody* root = mjs_asBody(mjs_firstChild(childWorld, mjOBJ_BODY, /*recurse=*/0));
            if (!root) {
                std::fprintf(stderr, "[scene] skipping %s: no root body\n", name.c_str());
                continue;
            }

            mjsFrame* frame = mjs_addFrame(worldbody, nullptr);
            frame->pos[0] = position[0];
            frame->pos[1] = position[1];
            frame->pos[2] = position[2];
            // YAML orientation is [x, y, z, w]; MuJoCo quaternions are [w, x, y, z].
            frame->quat[0] = orientation[3];
            frame->quat[1] = orientation[0];
            frame->quat[2] = orientation[1];
            frame->quat[3] = orientation[2];

            mjsElement* attached = mjs_attach(frame->element, root->element, (name + "/").c_str(), "");
            if (!attached) throw std::runtime_error("failed to attach " + name + ": " + mjs_getError(world));

            // Non-fixed objects get a floating base so they obey gravity and contacts.
            if (!fixed) {
                if (mjsBody* attachedBody = mjs_asBody(attached)) mjs_addFreeJoint(attachedBody);
            }
        }

        addActuators(world, actuated);

        mjModel* model = mj_compile(world, nullptr);
        std::string compileError = mjs_getError(world);

        for (mjSpec* child: children) mj_deleteSpec(child);
        mj_deleteSpec(world);

        if (!model) throw std::runtime_error("scene compile failed: " + compileError);
        return model;
    }

} // namespace mrover::sim

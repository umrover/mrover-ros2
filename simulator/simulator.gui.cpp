#include "simulator.hpp"

namespace mrover {

    auto r3ToBtVector3(R3d const& r3) -> btVector3 {
        return btVector3{static_cast<btScalar>(r3.x()), static_cast<btScalar>(r3.y()), static_cast<btScalar>(r3.z())};
    }

    auto Simulator::guiUpdate(wgpu::RenderPassEncoder& pass) -> void {
        if (mSaveTask.shouldUpdate() && mEnablePhysics) {
            if (auto it = mUrdfs.find("rover"); it != mUrdfs.end()) {
                URDF const& rover = it->second;

                SaveData save;
                save.baseTransform = rover.physics->getBaseWorldTransform();
                save.baseVelocity = rover.physics->getBaseVel();
                for (int link = 0; link < rover.physics->getNumLinks(); ++link) {
                    save.links.emplace_back(rover.physics->getJointPos(link), rover.physics->getJointVel(link));
                }
                mSaveHistory.push_back(std::move(save));
            }
        }

        ImGui_ImplWGPU_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        if (mEnablePhysics) {
            mSaveSelection = 0;
        } else if (int historySize = static_cast<int>(mSaveHistory.size()) - 1; historySize >= 0) {
            ImGui::Begin("History", nullptr, ImGuiWindowFlags_NoTitleBar);

            ImGui::SliderInt("History", &mSaveSelection, -historySize, 0);

            if (auto it = mUrdfs.find("rover"); it != mUrdfs.end()) {
                URDF const& rover = it->second;

                auto const& [baseTransform, baseVelocity, links] = mSaveHistory.at(mSaveSelection + historySize);
                rover.physics->setBaseWorldTransform(baseTransform);
                rover.physics->setBaseVel(baseVelocity);
                for (int link = 0; link < rover.physics->getNumLinks(); ++link) {
                    auto const& [position, velocity] = links.at(link);
                    rover.physics->setJointPos(link, position);
                    rover.physics->setJointVel(link, velocity);
                }

                btAlignedObjectArray<btQuaternion> q;
                btAlignedObjectArray<btVector3> m;
                rover.physics->forwardKinematics(q, m);
                rover.physics->updateCollisionObjectWorldTransforms(q, m);
            }

            ImGui::End();
        }

        // Draw fps text
        {
            ImGui::Begin("Settings", nullptr);
            ImGui::BeginDisabled(!mInGui);

            ImGui::SliderFloat("Target FPS", &mTargetUpdateRate, 5.0f, 1000.0f);
            ImGui::SliderFloat("Fly Speed", &mFlySpeed, 0.01f, 50.0f);
            ImGui::SliderFloat("Look Sense", &mLookSense, 0.0001f, 0.01f);
            ImGui::SliderFloat("FOV", &mFovDegrees, 10.0f, 120.0f);
            ImGui::InputFloat3("Gravity", mGravityAcceleration.m_floats);
            ImGui::SliderFloat("Rover Linear Speed", &mRoverLinearSpeed, 0.01f, 10.0f);
            ImGui::SliderFloat("Rover Angular Speed", &mRoverAngularSpeed, 0.01f, 10.0f);
            ImGui::Checkbox("Enable Physics (P)", &mEnablePhysics);
            ImGui::Checkbox("Render Models (M)", &mRenderModels);
            ImGui::Checkbox("Render Wireframe Colliders (C)", &mRenderWireframeColliders);
            ImGui::Text("Camera Locked: %s", mCameraInRoverTarget ? "True" : "False");
            ImGui::SliderFloat("Camera Lock Lerp", &mCameraLockSlerp, 0.0f, 1.0f);

            if (ImGui::BeginCombo("Sky Color", std::format("R: {:.3f} G: {:.3f} B: {:.3f} A: {:.3f}", mSkyColor[0], mSkyColor[1], mSkyColor[2], mSkyColor[3]).c_str(), ImGuiComboFlags_HeightLargest)) {
                ImGui::ColorPicker4("Sky Color", mSkyColor.data());
                ImGui::EndCombo();
            }

            // ImGui::SliderFloat("Float", &mFloat, 0.0f, 1000.0f);

            ImGui::Checkbox("Publish IK", &mPublishIk);
            if (mPublishIk) {
                if (ImGui::Checkbox("Arm Position Control", &mIkMode)) {
                    auto req = std::make_shared<srv::IkMode::Request>();
                    req->mode = mIkMode ? srv::IkMode::Request::POSITION_CONTROL : srv::IkMode::Request::VELOCITY_CONTROL;
                    mIkModeClient->async_send_request(req);
                }
                if (mIkMode) {
                    ImGui::SliderFloat("IK X Position", &mIkTarget.x(), 0, 1.5);
                    ImGui::SliderFloat("IK Y Position", &mIkTarget.y(), 0, .45);
                    ImGui::SliderFloat("IK Z Position", &mIkTarget.z(), -1.0, 1.0);
                    ImGui::SliderFloat("IK Pitch", &mIkPitch, -3.14f, 3.14f);
                    ImGui::SliderFloat("IK Roll", &mIkRoll, -3.14f, 3.14f);
                } else
                    ImGui::SliderFloat("Arm Speed", &mArmSpeed, 0, 1);
            }

            ImGui::InputDouble("Publish Hammer Distance Threshold", &mPublishHammerDistanceThreshold);
            ImGui::InputDouble("Publish Bottle Distance Threshold", &mPublishBottleDistanceThreshold);

            // for the imgui combo: https://skia.googlesource.com/external/github.com/ocornut/imgui/+/refs/tags/v1.73/imgui_demo.cpp
            static ImGuiComboFlags flags = 0;
            std::optional<std::string> selectedPath = std::nullopt;
            ImGui::Text("Map Selection:");
            if (ImGui::BeginCombo(" ", configFilename.c_str(), flags)) { // no text here, its taken care of above
                for (auto const& file: std::filesystem::directory_iterator{CONFIG_PATH}) {
                    if (ImGui::Selectable(file.path().filename().c_str())) {
                        selectedPath = std::make_optional<std::string>(file.path().filename());
                    }
                }
                ImGui::EndCombo();
            }

            if (selectedPath.has_value()) {
                std::cout << "Selected: " << selectedPath.value() << '\n';
                configFilename = selectedPath.value();
                initUrdfsFromParams(configFilename);
            }

            ImGui::EndDisabled();
            ImGui::End();
        }

        {
            ImGui::Begin("Telemtry", nullptr);

            ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
            if (auto it = mUrdfs.find("rover"); it != mUrdfs.end()) {
                URDF const& rover = it->second;

                {
                    SE3d baseLinkInMap = rover.linkInWorld("base_link");
                    R3d p = baseLinkInMap.translation();
                    S3d q = baseLinkInMap.quat();
                    ImGui::Text("Rover Position: (%.2f, %.2f, %.2f)", p.x(), p.y(), p.z());
                    ImGui::Text("Rover Orientation: (%.2f, %.2f, %.2f, %.2f)", q.w(), q.x(), q.y(), q.z());
                }
                {
                    R3d p = mCameraInWorld.translation();
                    S3d q = mCameraInWorld.quat();
                    ImGui::Text("Camera Position: (%.2f, %.2f, %.2f)", p.x(), p.y(), p.z());
                    ImGui::Text("Camera Orientation: (%.2f, %.2f, %.2f, %.2f)", q.w(), q.x(), q.y(), q.z());
                }
            }

            {
                // TODO(quintin): This does not work when the cursor is not in the middle of the screen...
                Eigen::Vector2d cursorInWindow;
                glfwGetCursorPos(mWindow.get(), &cursorInWindow.x(), &cursorInWindow.y());
                Eigen::Vector2i windowSize;
                glfwGetWindowSize(mWindow.get(), &windowSize.x(), &windowSize.y());
                Eigen::Vector2f cursorInNdc{
                        cursorInWindow.x() / windowSize.x() * 2 - 1,
                        1 - 2 * cursorInWindow.y() / windowSize.y(),
                };
                Eigen::Vector4f cursorInWorld = (mSceneUniforms.value.cameraToClip.matrix() * mSceneUniforms.value.worldToCamera.matrix()).inverse() * Eigen::Vector4f{cursorInNdc.x(), cursorInNdc.y(), NEAR, 1};
                cursorInWorld /= cursorInWorld.w();

                auto rayStart = btVector3{cursorInWorld.x(), cursorInWorld.y(), cursorInWorld.z()};
                auto rayEnd = rayStart + r3ToBtVector3(mCameraInWorld.rotation().matrix().col(0)) * 100;

                btMultiBodyDynamicsWorld::ClosestRayResultCallback rayCallback{rayStart, rayEnd};
                mDynamicsWorld->rayTest(rayStart, rayEnd, rayCallback);
                if (rayCallback.hasHit()) {
                    btVector3 const& hitPoint = rayCallback.m_hitPointWorld;
                    ImGui::Text("Cursor: (%.2f, %.2f, %.2f)", hitPoint.x(), hitPoint.y(), hitPoint.z());
                } else {
                    ImGui::Text("Cursor: None");
                }
            }

            for (Camera const& camera: mCameras) {
                float aspect = static_cast<float>(camera.resolution.x()) / static_cast<float>(camera.resolution.y());
                ImGui::Image(camera.colorTextureView, {320, 320 / aspect}, {0, 0}, {1, 1});
            }
            for (StereoCamera const& stereoCamera: mStereoCameras) {
                float aspect = static_cast<float>(stereoCamera.base.resolution.x()) / static_cast<float>(stereoCamera.base.resolution.y());
                ImGui::Image(stereoCamera.base.colorTextureView, {320, 320 / aspect}, {0, 0}, {1, 1});
            }

            ImGui::End();
        }

        ImGui::Render();
        ImGui_ImplWGPU_RenderDrawData(ImGui::GetDrawData(), pass);
    }

} // namespace mrover
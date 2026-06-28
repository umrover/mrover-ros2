#pragma once

#include <mujoco/mujoco.h>

struct GLFWwindow;

namespace mrover::sim {

    // The interactive window, built on MuJoCo's own OpenGL renderer (mjv/mjr) and GLFW.
    // Orbit/pan/zoom with the mouse; drive the rover with the keyboard. The simulator is
    // always run with this viewer (never headless), so glfw3 + OpenGL are required.
    class Viewer {
    public:
        explicit Viewer(mjModel* model);
        ~Viewer();

        Viewer(Viewer const&) = delete;
        auto operator=(Viewer const&) -> Viewer& = delete;
        Viewer(Viewer&&) = delete;
        auto operator=(Viewer&&) -> Viewer& = delete;

        [[nodiscard]] auto isOpen() const -> bool;
        auto render(mjModel* model, mjData* data) -> void;

        // Keyboard teleop axes, each in [-1, 1].
        [[nodiscard]] auto driveForward() const -> double { return (mKeyForward ? 1.0 : 0.0) - (mKeyBackward ? 1.0 : 0.0); }
        [[nodiscard]] auto driveTurn() const -> double { return (mKeyLeft ? 1.0 : 0.0) - (mKeyRight ? 1.0 : 0.0); }

    private:
        auto onMouseButton(int button, int action) -> void;
        auto onCursorMove(double xpos, double ypos) -> void;
        auto onScroll(double yoffset) -> void;
        auto onKey(int key, int action) -> void;

        mjModel* mModel = nullptr;
        GLFWwindow* mWindow = nullptr;

        mjvCamera mCamera{};
        mjvOption mOption{};
        mjvScene mScene{};
        mjrContext mContext{};

        bool mButtonLeft = false;
        bool mButtonRight = false;
        bool mButtonMiddle = false;
        bool mShift = false;
        double mLastX = 0.0;
        double mLastY = 0.0;

        bool mKeyForward = false;
        bool mKeyBackward = false;
        bool mKeyLeft = false;
        bool mKeyRight = false;
    };

} // namespace mrover::sim

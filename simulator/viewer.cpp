#include "viewer.hpp"

#include <cstdio>
#include <stdexcept>

#include <GLFW/glfw3.h>

namespace mrover::sim {

    namespace {
        // GLFW is C and only hands back the window, so route callbacks through the
        // window's user pointer to the owning Viewer.
        Viewer* self(GLFWwindow* window) { return static_cast<Viewer*>(glfwGetWindowUserPointer(window)); }
    } // namespace

    Viewer::Viewer(mjModel* model) : mModel{model} {
        if (!glfwInit()) throw std::runtime_error("failed to initialize GLFW");

        mWindow = glfwCreateWindow(1280, 720, "MRover Simulator", nullptr, nullptr);
        if (!mWindow) {
            glfwTerminate();
            throw std::runtime_error("failed to create GLFW window");
        }
        glfwMakeContextCurrent(mWindow);
        glfwSwapInterval(1);

        mjv_defaultCamera(&mCamera);
        mjv_defaultOption(&mOption);
        // Show the per-material visual meshes (group 2), hide the collision primitives
        // (group 3); everything else stays at its default visibility.
        mOption.geomgroup[2] = 1;
        mOption.geomgroup[3] = 0;
        mjv_defaultScene(&mScene);
        mjr_defaultContext(&mContext);
        mjv_makeScene(mModel, &mScene, 4000);
        mjr_makeContext(mModel, &mContext, mjFONTSCALE_150);

        // Start looking at the rover's spawn area from behind and above.
        mCamera.lookat[0] = 0.0;
        mCamera.lookat[1] = 0.0;
        mCamera.lookat[2] = 0.5;
        mCamera.distance = 6.0;
        mCamera.azimuth = 90.0;
        mCamera.elevation = -20.0;

        glfwSetWindowUserPointer(mWindow, this);
        glfwSetMouseButtonCallback(mWindow, [](GLFWwindow* w, int button, int action, int) {
            self(w)->onMouseButton(button, action);
        });
        glfwSetCursorPosCallback(mWindow, [](GLFWwindow* w, double x, double y) {
            self(w)->onCursorMove(x, y);
        });
        glfwSetScrollCallback(mWindow, [](GLFWwindow* w, double, double yoffset) {
            self(w)->onScroll(yoffset);
        });
        glfwSetKeyCallback(mWindow, [](GLFWwindow* w, int key, int, int action, int) {
            self(w)->onKey(key, action);
        });
    }

    Viewer::~Viewer() {
        mjr_freeContext(&mContext);
        mjv_freeScene(&mScene);
        if (mWindow) glfwDestroyWindow(mWindow);
        glfwTerminate();
    }

    auto Viewer::isOpen() const -> bool {
        return mWindow && !glfwWindowShouldClose(mWindow);
    }

    auto Viewer::render(mjModel* model, mjData* data) -> void {
        if (!mWindow) return;
        mjrRect viewport{0, 0, 0, 0};
        glfwGetFramebufferSize(mWindow, &viewport.width, &viewport.height);
        mjv_updateScene(model, data, &mOption, nullptr, &mCamera, mjCAT_ALL, &mScene);
        mjr_render(viewport, &mScene, &mContext);
        glfwSwapBuffers(mWindow);
        glfwPollEvents();
    }

    auto Viewer::onMouseButton(int button, int action) -> void {
        bool pressed = action == GLFW_PRESS;
        if (button == GLFW_MOUSE_BUTTON_LEFT) mButtonLeft = pressed;
        if (button == GLFW_MOUSE_BUTTON_RIGHT) mButtonRight = pressed;
        if (button == GLFW_MOUSE_BUTTON_MIDDLE) mButtonMiddle = pressed;
        glfwGetCursorPos(mWindow, &mLastX, &mLastY);
    }

    auto Viewer::onCursorMove(double xpos, double ypos) -> void {
        double dx = xpos - mLastX;
        double dy = ypos - mLastY;
        mLastX = xpos;
        mLastY = ypos;

        if (!mButtonLeft && !mButtonRight && !mButtonMiddle) return;

        int height = 1;
        glfwGetWindowSize(mWindow, nullptr, &height);

        mjtMouse action;
        if (mButtonRight) action = mShift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        else if (mButtonLeft) action = mShift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        else action = mjMOUSE_ZOOM;

        mjv_moveCamera(mModel, action, dx / height, dy / height, &mScene, &mCamera);
    }

    auto Viewer::onScroll(double yoffset) -> void {
        mjv_moveCamera(mModel, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &mScene, &mCamera);
    }

    auto Viewer::onKey(int key, int action) -> void {
        if (action == GLFW_REPEAT) return;
        bool pressed = action == GLFW_PRESS;

        switch (key) {
            case GLFW_KEY_ESCAPE:
            case GLFW_KEY_Q:
                if (pressed) glfwSetWindowShouldClose(mWindow, GLFW_TRUE);
                break;
            case GLFW_KEY_LEFT_SHIFT:
            case GLFW_KEY_RIGHT_SHIFT:
                mShift = pressed;
                break;
            case GLFW_KEY_W:
            case GLFW_KEY_UP:
                mKeyForward = pressed;
                break;
            case GLFW_KEY_S:
            case GLFW_KEY_DOWN:
                mKeyBackward = pressed;
                break;
            case GLFW_KEY_A:
            case GLFW_KEY_LEFT:
                mKeyLeft = pressed;
                break;
            case GLFW_KEY_D:
            case GLFW_KEY_RIGHT:
                mKeyRight = pressed;
                break;
            default:
                break;
        }
    }

} // namespace mrover::sim

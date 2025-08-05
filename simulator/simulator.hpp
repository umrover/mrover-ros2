#pragma once

#include "pch.hpp"

#include "glfw_pointer.hpp"
#include "wgpu_objects.hpp"

using namespace std::literals;

namespace mrover {

    using Clock = std::chrono::high_resolution_clock;

    constexpr static float DEG_TO_RAD = std::numbers::pi_v<float> / 180.0f;

    // Convert from ROS's right-handed +x forward, +y left, +z up to WGPU's left-handed +x right, +y up, +z backward
    static auto const ROS_TO_WGPU = (Eigen::Matrix4f{} << 0, -1, 0, 0, // WGPU x = -ROS y
                                     0, 0, 1, 0,                       // WGPU y = +ROS z
                                     1, 0, 0, 0,                       // WGPU z = +ROS x
                                     0, 0, 0, 1)
                                            .finished();

    static constexpr auto COLOR_FORMAT = wgpu::TextureFormat::BGRA8Unorm;
    static constexpr auto DEPTH_FORMAT = wgpu::TextureFormat::Depth32Float;
    static constexpr auto NORMAL_FORMAT = wgpu::TextureFormat::RGBA16Float;

    struct Camera;
    struct StereoCamera;
    class Simulator;

    // Eigen stores matrices in column-major which is the same as WGPU
    // As such there is no need to modify data before uploading to the device

    struct ModelUniforms {
        Eigen::Matrix4f modelToWorld{};
        Eigen::Matrix4f modelToWorldForNormals{};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct SceneUniforms {
        Eigen::Matrix4f worldToCamera{};
        Eigen::Matrix4f cameraToClip{};

        Eigen::Vector4f lightInWorld{};
        Eigen::Vector4f cameraInWorld{};
        Eigen::Vector4f lightColor{};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct ComputeUniforms {
        Eigen::Matrix4f clipToCamera{};
        Eigen::Vector2i resolution{};

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct Model {
        struct Mesh {
            SharedBuffer<Eigen::Vector3f> vertices;
            SharedBuffer<Eigen::Vector3f> normals;
            SharedBuffer<Eigen::Vector2f> uvs;
            SharedBuffer<std::uint32_t> indices;
            MeshTexture texture;
        };

        // DO NOT access the mesh unless you are certain it has been set from the async loader
        std::vector<Mesh> meshes;
        // https://en.cppreference.com/w/cpp/thread/future
        std::future<decltype(meshes)> asyncMeshesLoader;

        Model(Simulator& simulator, std::string_view uri);

        auto waitMeshes() -> void;

        [[nodiscard]] auto areMeshesReady() -> bool;

        ~Model();
    };

    struct URDF {
        struct LinkMeta {
            int index{};
            boost::container::small_vector<Uniform<ModelUniforms>, 2> visualUniforms;
            boost::container::small_vector<Uniform<ModelUniforms>, 2> collisionUniforms;
            Clock::time_point lastUpdate = Clock::now();
        };

        urdf::Model model;
        btMultiBody* physics = nullptr;
        std::unordered_map<std::string, LinkMeta> linkNameToMeta;

        URDF(Simulator& simulator, std::string_view uri, btTransform const& transform);

        static auto makeCollisionShapeForLink(Simulator& simulator, urdf::LinkConstSharedPtr const& link) -> btCollisionShape*;

        static auto makeCameraForLink(Simulator& simulator, btMultibodyLink const* link) -> Camera;

        [[nodiscard]] auto linkInWorld(std::string const& linkName) const -> SE3d;
    };

    struct PeriodicTask {
        Clock::duration period{};
        Clock::time_point lastUpdate = Clock::now();

        PeriodicTask() = default;

        explicit PeriodicTask(Clock::duration period)
            : period{period} {}

        explicit PeriodicTask(double rate)
            : PeriodicTask{std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>{1 / rate})} {}

        [[nodiscard]] auto shouldUpdate() -> bool {
            if (Clock::time_point now = Clock::now(); now - lastUpdate > period) {
                lastUpdate = now;
                return true;
            }

            return false;
        }
    };

    struct SensorBase {
        btMultibodyLink const* link = nullptr;
        PeriodicTask updateTask;
    };

    struct Camera : SensorBase {
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgPub;

        Eigen::Vector2i resolution;
        float fov{};
        std::string frameId;

        wgpu::Texture colorTexture;
        wgpu::TextureView colorTextureView;
        wgpu::Texture depthTexture;
        wgpu::TextureView depthTextureView;
        wgpu::Texture normalTexture;
        wgpu::TextureView normalTextureView;

        Uniform<SceneUniforms> sceneUniforms{};
        wgpu::BindGroup sceneBindGroup;

        wgpu::Buffer stagingBuffer;

        std::optional<wgpu::Future> future;
        bool needsMap = false;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct StereoCamera {
        Camera base;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcPub;

        wgpu::Buffer pointCloudStagingBuffer;
        wgpu::Buffer pointCloudBuffer;
        std::optional<wgpu::Future> pointCloudFuture;

        Uniform<ComputeUniforms> computeUniforms{};
        wgpu::BindGroup computeBindGroup;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct MotorGroup {
        std::vector<std::string> names;
        rclcpp::Subscription<msg::Throttle>::SharedPtr throttleSub;
        rclcpp::Subscription<msg::Velocity>::SharedPtr velocitySub;
        rclcpp::Subscription<msg::Position>::SharedPtr positionSub;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePub;
        rclcpp::Publisher<msg::ControllerState>::SharedPtr controllerStatePub;
    };

    // TODO(quintin): I removed IMU, you may want to add it back
    struct Imu : SensorBase {
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magPub;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr uncalibPub, imuPub;
        rclcpp::Publisher<mrover::msg::CalibrationStatus>::SharedPtr calibStatusPub;
        // ros::Publisher<sensor_msgs::msg::MagneticField> magPub, uncalibPub, calibStatusPub;
    };

    struct Gps : SensorBase {
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fixPub;
    };

    class Simulator final : public rclcpp::Node {
        friend Model;
        friend URDF;

        // Settings

        float mTargetUpdateRate = 180;
        bool mHeadless = false;
        int mQuitKey = GLFW_KEY_Q;
        int mInGuiKey = GLFW_KEY_ESCAPE;
        int mCamRightKey = GLFW_KEY_D;
        int mCamLeftKey = GLFW_KEY_A;
        int mCamForwardKey = GLFW_KEY_W;
        int mCamBackwardKey = GLFW_KEY_S;
        int mCamUpKey = GLFW_KEY_SPACE;
        int mCamDownKey = GLFW_KEY_LEFT_CONTROL;
        int mRoverRightKey = GLFW_KEY_L;
        int mRoverLeftKey = GLFW_KEY_J;
        int mRoverForwardKey = GLFW_KEY_I;
        int mRoverBackwardKey = GLFW_KEY_COMMA;
        int mRoverStopKey = GLFW_KEY_K;
        int mTogglePhysicsKey = GLFW_KEY_P;
        int mToggleRenderModelsKey = GLFW_KEY_M;
        int mToggleRenderWireframeCollidersKey = GLFW_KEY_C;
        int mToggleCameraLockKey = GLFW_KEY_O;

        float mFlySpeed = 5.0f;
        float mRoverLinearSpeed = 1.0f;
        float mRoverAngularSpeed = 1.5f;
        float mLookSense = 0.004f;
        float mFovDegrees = 60.0f;
        btVector3 mGravityAcceleration{0.0f, 0.0f, -9.81f};
        bool mEnablePhysics{};
        bool mRenderModels = true;
        bool mRenderWireframeColliders = false;
        double mPublishHammerDistanceThreshold = 3;
        double mPublishBottleDistanceThreshold = 3;
        float mCameraLockSlerp = 0.02;

        float mFloat = 0.0f;

        // ROS

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mGroundTruthPub;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mCmdVelPub;

        rclcpp::Publisher<msg::ImageTargets>::SharedPtr mImageTargetsPub;

        tf2_ros::Buffer mTfBuffer{get_clock()};
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster{this};

        bool mPublishIk = true;
        Eigen::Vector3f mIkTarget{0.382, 0.01, -0.217};
        rclcpp::Publisher<msg::IK>::SharedPtr mIkTargetPub;

        R3d mGpsLinearizationReferencePoint{};
        double mGpsLinerizationReferenceHeading{};

        std::default_random_engine mRNG{std::random_device{}()};
        std::normal_distribution<> mGPSDist{0, 0.02},
                mAccelDist{0, 0.01},
                mGyroDist{0, 0.01},
                mMagDist{0, 0.1},
                mRollDist{0, 0.01},
                mPitchDist{0, 0.01},
                mYawDist{0, 0.01};

        // drift rate in rad/minute about each axis
        // R3 mOrientationDriftRate{0.0, 0.0, 1.0};
        R3d mOrientationDriftRate = R3d::Zero();
        R3d mOrientationDrift = R3d::Zero();

        bool mIsHeadless{};

        int64_t mMotorTimeoutMs{};

        // Rendering

        GlfwInstance mGlfwInstance;
        GlfwPointer<GLFWwindow, glfwCreateWindow, glfwDestroyWindow> mWindow;
        wgpu::Instance mWgpuInstance;
        wgpu::Surface mSurface;
        wgpu::Adapter mAdapter;
        wgpu::Device mDevice;
        std::unique_ptr<wgpu::ErrorCallback> mErrorCallback;
        wgpu::Queue mQueue;
        wgpu::Texture mDepthTexture;
        wgpu::TextureView mDepthTextureView;
        wgpu::Texture mNormalTexture;
        wgpu::TextureView mNormalTextureView;

        wgpu::ShaderModule mShaderModule;
        wgpu::RenderPipeline mPbrPipeline;
        wgpu::RenderPipeline mWireframePipeline;

        wgpu::ComputePipeline mPointCloudPipeline;

        std::unordered_map<std::string, Model> mUriToModel;

        bool mHasFocus = false;
        bool mInGui = false;

        Uniform<SceneUniforms> mSceneUniforms;

        Eigen::Vector4f mSkyColor{0.05f, 0.8f, 0.92f, 1.0f};

        // Physics

        std::unique_ptr<btDefaultCollisionConfiguration> mCollisionConfig;
        std::unique_ptr<btCollisionDispatcher> mDispatcher;
        std::unique_ptr<btHashedOverlappingPairCache> mOverlappingPairCache;
        std::unique_ptr<btDbvtBroadphase> mBroadphase;
        std::unique_ptr<btMultiBodyConstraintSolver> mSolver;
        std::unique_ptr<btMultiBodyDynamicsWorld> mDynamicsWorld;
        std::vector<std::unique_ptr<btCollisionShape>> mCollisionShapes;
        std::vector<std::unique_ptr<btMultiBody>> mMultiBodies;
        std::vector<std::unique_ptr<btMultiBodyLinkCollider>> mMultibodyCollider;
        std::vector<std::unique_ptr<btMultiBodyConstraint>> mMultibodyConstraints;

        std::unordered_map<btCollisionShape*, std::string> mMeshToUri;

        struct SaveData {
            struct LinkData {
                btScalar position{};
                btScalar velocity{};
            };

            btTransform baseTransform;
            btVector3 baseVelocity;
            boost::container::small_vector<LinkData, 32> links;
        };

        int mSaveSelection = 0;
        PeriodicTask mSaveTask;
        boost::circular_buffer<SaveData> mSaveHistory;

        template<typename T, typename... Args>
        auto makeBulletObject(auto& vector, Args&&... args) -> T* {
            auto pointer = std::make_unique<T>(std::forward<Args>(args)...);
            auto* rawPointer = pointer.get();
            vector.emplace_back(std::move(pointer));
            return rawPointer;
        }

        R3d mRoverLinearVelocity{};

        // Scene

        std::unordered_map<std::string, URDF> mUrdfs;

        auto getUrdf(std::string const& name) -> std::optional<std::reference_wrapper<URDF>>;

        SE3d mCameraInWorld{R3d{-3.0, 0.0, 1.5}, SO3d::Identity()};

        std::optional<SE3d> mCameraInRoverTarget;

        std::vector<StereoCamera> mStereoCameras;
        std::vector<Camera> mCameras;
        std::vector<Imu> mImus;
        std::vector<Gps> mGps;
        std::vector<MotorGroup> mMotorGroups;

        static constexpr float NEAR = 0.1f;
        static constexpr float FAR = 1000.0f;

        boost::bimap<std::string, std::string> mMsgToUrdf;

        template<typename F, typename N, typename V>
        auto forEachMotor(N const& names, V const& values, F&& function) -> void {
            if (names.size() != values.size()) {
                RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Received different number of joint names and values. Ignoring...");
                return;
            }

            if (auto it = mUrdfs.find("rover"); it != mUrdfs.end()) {
                URDF& rover = it->second;

                for (std::size_t i = 0; i < names.size(); ++i) {
                    std::string const& name = names[i];
                    float value = values[i];

                    auto it = mMsgToUrdf.left.find(name);
                    if (it == mMsgToUrdf.left.end()) {
                        RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, std::format("Unknown joint name: {}. Either the wrong name was sent OR the simulator does not yet support it", name));
                        continue;
                    }

                    std::string const& urdfName = it->second;
                    URDF::LinkMeta& linkMeta = rover.linkNameToMeta.at(urdfName);
                    linkMeta.lastUpdate = Clock::now();

                    auto* motor = std::bit_cast<btMultiBodyJointMotor*>(rover.physics->getLink(linkMeta.index).m_userPtr);
                    assert(motor);
                    function(motor, value);
                }
            }
        }

        // Other

        Clock::time_point mLastTickTime = Clock::now();

        LoopProfiler mLoopProfiler{get_logger(), 512};

        auto renderCamera(Camera& camera, wgpu::CommandEncoder& encoder, wgpu::RenderPassDescriptor const& passDescriptor) -> void;

        auto computeStereoCamera(StereoCamera& stereoCamera, wgpu::CommandEncoder& encoder) -> void;

        auto gpsAndImusUpdate(Clock::duration dt) -> void;

        auto motorStatusUpdate() -> void;

        auto linksToTfUpdate() -> void;

        auto keyCallback(int key, int scancode, int action, int mods) -> void;

        auto frameBufferResizedCallback(int width, int height) -> void;

        auto positionsCallback(msg::Position::ConstSharedPtr const& msg) -> void;

        auto velocitiesCallback(msg::Velocity::ConstSharedPtr const& msg) -> void;

        auto throttlesCallback(msg::Throttle::ConstSharedPtr const& msg) -> void;

    public:
        Simulator();

        ~Simulator() override;

        Simulator(Simulator const&) = delete;
        Simulator(Simulator&&) = delete;

        auto operator=(Simulator const&) -> Simulator& = delete;
        auto operator=(Simulator&&) -> Simulator& = delete;

        auto initWindow() -> void;

        auto initRender() -> void;

        auto renderModel(wgpu::RenderPassEncoder& pass, Model& model, Uniform<ModelUniforms>& uniforms, SIM3 const& modelToWorld, bool isRoverCamera = false) -> void;

        auto initPhysics() -> void;

        auto initUrdfsFromParams() -> void;

        auto tick() -> void;

        auto centerCursor() const -> void;

        auto freeLook(Clock::duration dt) -> void;

        auto cameraLock(Clock::duration dt) -> void;

        auto setCameraInRoverTarget() -> void;

        auto userControls(Clock::duration dt) -> void;

        auto renderModels(wgpu::RenderPassEncoder& pass) -> void;

        auto renderWireframeColliders(wgpu::RenderPassEncoder& pass) -> void;

        auto renderUpdate() -> void;

        auto camerasUpdate(wgpu::CommandEncoder encoder,
                           wgpu::RenderPassColorAttachment& colorAttachment,
                           wgpu::RenderPassColorAttachment& normalAttachment,
                           wgpu::RenderPassDepthStencilAttachment& depthStencilAttachment,
                           wgpu::RenderPassDescriptor const& renderPassDescriptor) -> void;

        auto guiUpdate(wgpu::RenderPassEncoder& pass) -> void;

        auto physicsUpdate(Clock::duration dt) -> void;

        auto makeTextureAndView(int width, int height, wgpu::TextureFormat const& format, wgpu::TextureUsage const& usage, wgpu::TextureAspect const& aspect) -> std::pair<wgpu::Texture, wgpu::TextureView>;

        auto makeFramebuffers(int width, int height) -> void;

        auto makeRenderPipelines() -> void;

        auto computeNavSatFix(R3d const& gpsInMap, R3d const& referenceGeodetic, double referenceHeadingDegrees) -> sensor_msgs::msg::NavSatFix;

        auto computeImu(SO3d const& imuInMap, R3d const& imuAngularVelocity, R3d const& linearAcceleration) -> sensor_msgs::msg::Imu;
    };

    auto uriToPath(std::string_view uri) -> std::filesystem::path;

    auto performXacro(std::filesystem::path const& path) -> std::string;

    auto readTextFile(std::filesystem::path const& path) -> std::string;

    auto readTexture(std::filesystem::path const& textureFileName) -> cv::Mat;

    auto urdfPoseToBtTransform(urdf::Pose const& pose) -> btTransform;

    auto btTransformToSe3(btTransform const& transform) -> SE3d;

    auto computeCameraToClip(float fovY, float aspect, float zNear, float zFar) -> Eigen::Matrix4f;

} // namespace mrover

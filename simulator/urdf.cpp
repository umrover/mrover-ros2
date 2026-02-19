#include "simulator.hpp"

namespace mrover {

    constexpr float MASS_MULTIPLIER = 1.0f;
    constexpr float DISTANCE_MULTIPLIER = 1.0f;
    constexpr float INERTIA_MULTIPLIER = MASS_MULTIPLIER * DISTANCE_MULTIPLIER * DISTANCE_MULTIPLIER;

    auto urdfPosToBtPos(urdf::Vector3 const& vec) -> btVector3 {
        return btVector3{static_cast<btScalar>(vec.x), static_cast<btScalar>(vec.y), static_cast<btScalar>(vec.z)} * DISTANCE_MULTIPLIER;
    }

    auto urdfQuatToBtQuat(urdf::Rotation const& quat) -> btQuaternion {
        return btQuaternion{static_cast<btScalar>(quat.x), static_cast<btScalar>(quat.y), static_cast<btScalar>(quat.z), static_cast<btScalar>(quat.w)};
    }

    auto urdfPoseToBtTransform(urdf::Pose const& pose) -> btTransform {
        return btTransform{urdfQuatToBtQuat(pose.rotation), urdfPosToBtPos(pose.position)};
    }

    auto urdfDistToBtDist(double scalar) -> btScalar {
        return static_cast<btScalar>(scalar) * DISTANCE_MULTIPLIER;
    }

    auto urdfMassToBtMass(double scalar) -> btScalar {
        return static_cast<btScalar>(scalar) * MASS_MULTIPLIER;
    }

    auto urdfInertiaToBtInertia(urdf::InertialSharedPtr const& inertia) -> btVector3 {
        return btVector3{static_cast<btScalar>(inertia->ixx), static_cast<btScalar>(inertia->iyy), static_cast<btScalar>(inertia->izz)} * INERTIA_MULTIPLIER;
    }

    auto Simulator::initUrdfsFromParams() -> void {
        {
            // As far as I can tell array of structs in YAML is not even supported by ROS 2 as compared to ROS 1
            // See: https://robotics.stackexchange.com/questions/109909/reading-a-vector-of-structs-as-parameters-in-ros2

            std::map<std::string, rclcpp::Parameter> objects;
            get_parameters("objects", objects);

            // Extract the names of the objects, there will be multiple object_name.* keys that we consolidate into just object_name
            std::set<std::string> names;
            std::ranges::transform(objects | std::views::keys, std::inserter(names, names.end()), [](std::string const& fullName) {
                return fullName.substr(0, fullName.find('.'));
            });

            for (auto const& name: names) {
                std::string uri = objects.at(std::format("{}.uri", name)).as_string();
                std::vector<double> position = objects.contains(std::format("{}.position", name)) ? objects.at(std::format("{}.position", name)).as_double_array() : std::vector<double>{0, 0, 0};
                std::vector<double> orientation = objects.contains(std::format("{}.orientation", name)) ? objects.at(std::format("{}.orientation", name)).as_double_array() : std::vector<double>{0, 0, 0, 1};
                if (position.size() != 3) throw std::invalid_argument{"Position must have 3 elements"};
                if (orientation.size() != 4) throw std::invalid_argument{"Orientation must have 4 elements"};
                btTransform transform{btQuaternion{static_cast<btScalar>(orientation[0]), static_cast<btScalar>(orientation[1]), static_cast<btScalar>(orientation[2]), static_cast<btScalar>(orientation[3])}, btVector3{static_cast<btScalar>(position[0]), static_cast<btScalar>(position[1]), static_cast<btScalar>(position[2])}};
                if (auto [_, wasAdded] = mUrdfs.try_emplace(name, *this, uri, transform); !wasAdded) {
                    throw std::invalid_argument{std::format("Duplicate object name: {}", name)};
                }
            }
        }
    }

    auto URDF::makeCollisionShapeForLink(Simulator& simulator, urdf::LinkConstSharedPtr const& link) -> btCollisionShape* {
        boost::container::small_vector<std::pair<btCollisionShape*, btTransform>, 4> shapes;
        for (urdf::CollisionSharedPtr const& collision: link->collision_array) {
            if (!collision->geometry) throw std::invalid_argument{"Collision has no geometry"};

            switch (collision->geometry->type) {
                case urdf::Geometry::BOX: {
                    auto box = std::dynamic_pointer_cast<urdf::Box>(collision->geometry);
                    assert(box);

                    btVector3 boxHalfExtents = urdfPosToBtPos(box->dim) / 2;
                    shapes.emplace_back(simulator.makeBulletObject<btBoxShape>(simulator.mCollisionShapes, boxHalfExtents), urdfPoseToBtTransform(collision->origin));
                    break;
                }
                case urdf::Geometry::SPHERE: {
                    auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(collision->geometry);
                    assert(sphere);

                    btScalar radius = urdfDistToBtDist(sphere->radius);
                    shapes.emplace_back(simulator.makeBulletObject<btSphereShape>(simulator.mCollisionShapes, radius), urdfPoseToBtTransform(collision->origin));
                    break;
                }
                case urdf::Geometry::CYLINDER: {
                    auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(collision->geometry);
                    assert(cylinder);

                    btScalar radius = urdfDistToBtDist(cylinder->radius);
                    btScalar halfLength = urdfDistToBtDist(cylinder->length) / 2;
                    btVector3 cylinderHalfExtents{radius, radius, halfLength};
                    shapes.emplace_back(simulator.makeBulletObject<btCylinderShapeZ>(simulator.mCollisionShapes, cylinderHalfExtents), urdfPoseToBtTransform(collision->origin));
                    break;
                }
                case urdf::Geometry::MESH: {
                    auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(collision->geometry);

                    assert(mesh);

                    std::string const& fileUri = mesh->filename;
                    auto [it, wasInserted] = simulator.mUriToModel.try_emplace(fileUri, simulator, fileUri);
                    Model& model = it->second;

                    model.waitMeshes();

                    // Colliders with non-zero mass must be convex
                    // This is a limitation of most physics engines
                    if (link->inertial->mass > std::numeric_limits<float>::epsilon()) {
                        auto* convexHullShape = simulator.makeBulletObject<btConvexHullShape>(simulator.mCollisionShapes);
                        for (Model::Mesh const& meshData: model.meshes) {
                            for (Eigen::Vector3f const& point: meshData.vertices.data) {
                                convexHullShape->addPoint(btVector3{point.x(), point.y(), point.z()}, false);
                            }
                        }
                        convexHullShape->recalcLocalAabb();
                        shapes.emplace_back(convexHullShape, urdfPoseToBtTransform(collision->origin));
                        simulator.mMeshToUri.emplace(convexHullShape, fileUri);
                    } else {
                        auto* triangleMesh = new btTriangleMesh{};
                        triangleMesh->preallocateVertices(static_cast<int>(model.meshes.front().vertices.data.size()));
                        triangleMesh->preallocateIndices(static_cast<int>(model.meshes.front().indices.data.size()));

                        for (Model::Mesh const& meshData: model.meshes) {
                            for (std::size_t i = 0; i < meshData.indices.data.size(); i += 3) {
                                Eigen::Vector3f v0 = meshData.vertices.data[meshData.indices.data[i + 0]];
                                Eigen::Vector3f v1 = meshData.vertices.data[meshData.indices.data[i + 1]];
                                Eigen::Vector3f v2 = meshData.vertices.data[meshData.indices.data[i + 2]];
                                triangleMesh->addTriangle(btVector3{v0.x(), v0.y(), v0.z()}, btVector3{v1.x(), v1.y(), v1.z()}, btVector3{v2.x(), v2.y(), v2.z()});
                            }
                        }

                        auto* meshShape = simulator.makeBulletObject<btBvhTriangleMeshShape>(simulator.mCollisionShapes, triangleMesh, true);
                        meshShape->setMargin(0.01);
                        shapes.emplace_back(meshShape, urdfPoseToBtTransform(collision->origin));
                        simulator.mMeshToUri.emplace(meshShape, fileUri);
                    }
                    break;
                }
                default:
                    throw std::invalid_argument{"Unsupported collision type"};
            }
        }
        btCollisionShape* finalShape;
        switch (shapes.size()) {
            case 0:
                finalShape = simulator.makeBulletObject<btEmptyShape>(simulator.mCollisionShapes);
                break;
            default:
                auto* compoundShape = simulator.makeBulletObject<btCompoundShape>(simulator.mCollisionShapes);
                for (auto const& [shape, transform]: shapes) {
                    compoundShape->addChildShape(transform, shape);
                }
                finalShape = compoundShape;
                break;
        }
        return finalShape;
    }

    auto URDF::makeCameraForLink(Simulator& simulator, btMultibodyLink const* link) -> Camera {
        Camera camera;
        camera.link = link;
        camera.resolution = {640, 480};
        camera.updateTask = PeriodicTask{20};
        // TODO(quintin): Why do I have to cast this
        wgpu::TextureUsage usage = static_cast<wgpu::TextureUsage::W>(wgpu::TextureUsage::RenderAttachment | wgpu::TextureUsage::TextureBinding);
        wgpu::TextureUsage colorUsage = static_cast<wgpu::TextureUsage::W>(usage | wgpu::TextureUsage::CopySrc);
        std::tie(camera.colorTexture, camera.colorTextureView) = simulator.makeTextureAndView(camera.resolution.x(), camera.resolution.y(), COLOR_FORMAT, colorUsage, wgpu::TextureAspect::All);
        std::tie(camera.normalTexture, camera.normalTextureView) = simulator.makeTextureAndView(camera.resolution.x(), camera.resolution.y(), NORMAL_FORMAT, usage, wgpu::TextureAspect::All);
        std::tie(camera.depthTexture, camera.depthTextureView) = simulator.makeTextureAndView(camera.resolution.x(), camera.resolution.y(), DEPTH_FORMAT, usage, wgpu::TextureAspect::DepthOnly);
        return camera;
    }

    URDF::URDF(Simulator& simulator, std::string_view uri, btTransform const& transform) {
        RCLCPP_INFO_STREAM(simulator.get_logger(), std::format("{}", performXacro(uriToPath(uri))));
        if (!model.initString(performXacro(uriToPath(uri)))) throw std::runtime_error{std::format("Failed to parse URDF from URI: {}", uri)};

        std::size_t multiBodyLinkCount = model.links_.size() - 1; // Root link is treated separately by multibody, so subtract it off
        auto* multiBody = physics = simulator.makeBulletObject<btMultiBody>(simulator.mMultiBodies, multiBodyLinkCount, 0, btVector3{0, 0, 0}, false, false);
        multiBody->setBaseWorldTransform(transform);

        std::vector<btMultiBodyLinkCollider*> collidersToFinalize;
        std::vector<btMultiBodyConstraint*> constraintsToFinalize;

        // NOLINTNEXTLINE(misc-no-recursion)
        auto traverse = [&](auto&& self, urdf::LinkConstSharedPtr const& link) -> void {
            RCLCPP_INFO_STREAM(simulator.get_logger(), std::format("Processing link: {}", link->name));

            auto linkIndex = static_cast<int>(linkNameToMeta.size()) - 1;
            auto [it, was_inserted] = linkNameToMeta.emplace(link->name, linkIndex);
            assert(was_inserted);

            // TODO(quintin): Configure this from a plugins XML file?
            if (link->name.contains("camera"sv)) {
                Camera camera = makeCameraForLink(simulator, &multiBody->getLink(linkIndex));
                if (link->name.contains("zed"sv)) {
                    std::string frameId, imageTopic, pointCloudTopic;
                    if (link->name.contains("zed_mini")) {
                        frameId = "zed_mini_left_camera_frame";
                        imageTopic = "mast_camera/left/image";
                        pointCloudTopic = "mast_camera/left/points";
                    } else {
                        frameId = "zed_left_camera_frame";
                        imageTopic = "zed/left/image";
                        pointCloudTopic = "zed/left/points";
                    }
                    camera.frameId = frameId;
                    camera.imgPub = simulator.create_publisher<sensor_msgs::msg::Image>(imageTopic, 1);
                    camera.fov = 60;
                    StereoCamera stereoCamera;
                    stereoCamera.base = std::move(camera);
                    stereoCamera.pcPub = simulator.create_publisher<sensor_msgs::msg::PointCloud2>(pointCloudTopic, 1);
                    simulator.mStereoCameras.emplace_back(std::move(stereoCamera));
                } else {
                    camera.frameId = "long_range_camera_link";
                    camera.imgPub = simulator.create_publisher<sensor_msgs::msg::Image>("long_range_cam/image", 1);
                    camera.fov = 15;
                    simulator.mCameras.push_back(std::move(camera));
                }
            } else if (link->name.contains("imu"sv)) {
                // TODO(quintin): I removed IMU, you may want to add it back
                Imu& imu = simulator.mImus.emplace_back();
                imu.link = &multiBody->getLink(linkIndex);
                imu.updateTask = PeriodicTask{50};
                imu.imuPub = simulator.create_publisher<sensor_msgs::msg::Imu>("imu/data", 1);
                imu.magPub = simulator.create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 1);
                imu.uncalibPub = simulator.create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 1);
                imu.calibStatusPub = simulator.create_publisher<mrover::msg::CalibrationStatus>("imu/calibration", 1);
            } else if (link->name.contains("gps"sv)) {
                Gps& gps = simulator.mGps.emplace_back();
                gps.link = &multiBody->getLink(linkIndex);
                gps.updateTask = PeriodicTask{10};
                std::string topic;
                if (link->name.contains("left")) {
                    topic = "right_gps/fix";
                } else if (link->name.contains("right")) {
                    topic = "left_gps/fix";
                } else {
                    topic = "gps/fix";
                }
                gps.fixPub = simulator.create_publisher<sensor_msgs::msg::NavSatFix>(topic, 1);
            }

            for (urdf::VisualSharedPtr const& visual: link->visual_array) {
                switch (visual->geometry->type) {
                    case urdf::Geometry::MESH: {
                        auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry);
                        std::string const& fileUri = mesh->filename;
                        simulator.mUriToModel.try_emplace(fileUri, simulator, fileUri);
                        break;
                    }
                    default: {
                        RCLCPP_WARN_STREAM(simulator.get_logger(), "Currently only mesh visuals are supported");
                        break;
                    }
                }
            }

            auto* collider = simulator.makeBulletObject<btMultiBodyLinkCollider>(simulator.mMultibodyCollider, multiBody, linkIndex);
            collider->setFriction(1);
            collidersToFinalize.push_back(collider);
            collider->setCollisionShape(makeCollisionShapeForLink(simulator, link));
            simulator.mDynamicsWorld->addCollisionObject(collider, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);

            btScalar mass = 1;
            btVector3 inertia{1, 1, 1}; // TODO(quintin): Is this a sane default?
            if (link->inertial) {
                mass = urdfMassToBtMass(link->inertial->mass);
                inertia = urdfInertiaToBtInertia(link->inertial);
            }

            if (urdf::JointConstSharedPtr parentJoint = link->parent_joint) {
                int parentIndex = linkNameToMeta.at(parentJoint->parent_link_name).index;
                btTransform jointInParent = urdfPoseToBtTransform(parentJoint->parent_to_joint_origin_transform);
                btTransform comInJoint = link->inertial ? urdfPoseToBtTransform(link->inertial->origin) : btTransform::getIdentity();
                btVector3 axisInJoint = urdfPosToBtPos(parentJoint->axis);

                switch (parentJoint->type) {
                    case urdf::Joint::FIXED: {
                        RCLCPP_INFO_STREAM(simulator.get_logger(), std::format("Fixed joint {}: {} <-> {}", parentJoint->name, parentJoint->parent_link_name, parentJoint->child_link_name));
                        multiBody->setupFixed(linkIndex, mass, inertia, parentIndex, jointInParent.getRotation().inverse(), jointInParent.getOrigin(), comInJoint.getOrigin());
                        break;
                    }
                    case urdf::Joint::CONTINUOUS:
                    case urdf::Joint::REVOLUTE: {
                        RCLCPP_INFO_STREAM(simulator.get_logger(), std::format("Rotating joint {}: {} <-> {}", parentJoint->name, parentJoint->parent_link_name, parentJoint->child_link_name));
                        multiBody->setupRevolute(linkIndex, mass, inertia, parentIndex, jointInParent.getRotation().inverse(), axisInJoint, jointInParent.getOrigin(), comInJoint.getOrigin(), true);

                        if (link->name.contains("wheel"sv)) {
                            RCLCPP_INFO_STREAM(simulator.get_logger(), "\tWheel");

                            collider->setRollingFriction(0.0);
                            collider->setSpinningFriction(0.0);
                            collider->setFriction(0.8);
                            collider->setContactStiffnessAndDamping(30000, 1000);
                        }
                        break;
                    }
                    case urdf::Joint::PRISMATIC: {
                        RCLCPP_INFO_STREAM(simulator.get_logger(), std::format("Prismatic joint {}: {} <-> {}", parentJoint->name, parentJoint->parent_link_name, parentJoint->child_link_name));
                        multiBody->setupPrismatic(linkIndex, mass, inertia, parentIndex, jointInParent.getRotation().inverse(), axisInJoint, jointInParent.getOrigin(), comInJoint.getOrigin(), true);
                        break;
                    }
                    default:
                        throw std::invalid_argument{"Unsupported joint type"};
                }
                switch (parentJoint->type) {
                    case urdf::Joint::CONTINUOUS:
                    case urdf::Joint::REVOLUTE:
                    case urdf::Joint::PRISMATIC: {
                        RCLCPP_INFO_STREAM(simulator.get_logger(), "\tMotor");
                        auto* motor = simulator.makeBulletObject<btMultiBodyJointMotor>(simulator.mMultibodyConstraints, multiBody, linkIndex, 0, 0);
                        constraintsToFinalize.push_back(motor);
                        multiBody->getLink(linkIndex).m_userPtr = motor;
                        break;
                    }
                    default:
                        break;
                }
                switch (parentJoint->type) {
                    case urdf::Joint::REVOLUTE:
                    case urdf::Joint::PRISMATIC: {
                        auto lower = static_cast<btScalar>(parentJoint->limits->lower), upper = static_cast<btScalar>(parentJoint->limits->upper);
                        auto* limitConstraint = simulator.makeBulletObject<btMultiBodyJointLimitConstraint>(simulator.mMultibodyConstraints, multiBody, linkIndex, lower, upper);
                        constraintsToFinalize.push_back(limitConstraint);
                    }
                    default:
                        break;
                }

                multiBody->getLink(linkIndex).m_collider = collider; // Bullet WHY? Why is this not exposed via a function call? This took a LONG time to figure out btw.
            } else {
                multiBody->setBaseMass(mass);
                multiBody->setBaseInertia(inertia);
                multiBody->setBaseCollider(collider);
            }

            it->second.collisionUniforms.resize(link->collision_array.size());
            it->second.visualUniforms.resize(link->visual_array.size());

            for (urdf::LinkConstSharedPtr childLink: link->child_links) {
                self(self, childLink);
            }
        };

        traverse(traverse, model.getRoot());

        multiBody->finalizeMultiDof();
        btAlignedObjectArray<btQuaternion> q;
        btAlignedObjectArray<btVector3> m;
        multiBody->forwardKinematics(q, m);
        multiBody->updateCollisionObjectWorldTransforms(q, m);
        simulator.mDynamicsWorld->addMultiBody(multiBody);

        for (btMultiBodyConstraint* constraint: constraintsToFinalize) {
            constraint->finalizeMultiDof();
            simulator.mDynamicsWorld->addMultiBodyConstraint(constraint);
        }
    }

    auto Simulator::getUrdf(std::string const& name) -> std::optional<std::reference_wrapper<URDF>> {
        auto it = mUrdfs.find(name);
        if (it == mUrdfs.end()) return std::nullopt;

        return it->second;
    }

} // namespace mrover

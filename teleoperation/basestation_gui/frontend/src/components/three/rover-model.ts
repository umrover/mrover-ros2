import * as THREE from 'three'
import GUI from 'lil-gui'
import URDFLoader from 'urdf-loader'
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js'
import { DRACOLoader } from 'three/addons/loaders/DRACOLoader.js'

export interface JointUpdate {
  name: string
  position: number
}

export interface Position3D {
  x: number
  y: number
  z: number
}

export interface RoverModel {
  updateJoints: (joints: JointUpdate[]) => void
  updateIKTarget: (position: Position3D | null) => void
}

const DEFAULT_JOINT_VALUES: Record<string, number> = {
  chassis_to_arm_a: 24.14,
  arm_a_to_arm_b: -0.785,
  arm_b_to_arm_c: 1.91,
  arm_c_to_arm_d: -1,
  arm_d_to_arm_e: -1.57,
  gripper_link: 0,
}

export function loadRover(parent: THREE.Group): RoverModel {
  let rover: THREE.Object3D | null = null

  const ikSphere = new THREE.Mesh(
    new THREE.SphereGeometry(2, 16, 16),
    new THREE.MeshStandardMaterial({
      color: 0xff0000,
      emissive: 0xff0000,
      emissiveIntensity: 0.3,
    }),
  )
  ikSphere.visible = false
  parent.add(ikSphere)

  // Debug GUI (hidden by default)
  const gui = new GUI({ width: 400 })
  gui.hide()

  // URDF loader with GLTF + Draco mesh support
  const manager = new THREE.LoadingManager()
  const loader = new URDFLoader(manager)
  loader.packages = { mrover: '/urdf' }

  loader.loadMeshCb = function (
    path: string,
    manager: THREE.LoadingManager,
    onComplete: (mesh: THREE.Object3D, err?: Error) => void,
  ) {
    const gltfLoader = new GLTFLoader(manager)
    const dracoLoader = new DRACOLoader()
    dracoLoader.setDecoderPath('https://www.gstatic.com/draco/v1/decoders/')
    gltfLoader.setDRACOLoader(dracoLoader)
    gltfLoader.load(
      path,
      result => onComplete(result.scene),
      undefined,
      err => onComplete(new THREE.Object3D(), err as Error),
    )
  }

  loader.load(
    '/urdf/rover/rover.urdf',
    (robot: THREE.Object3D) => {
      rover = robot

      // ROS Z-up to Three.js Y-up conversion
      const roverContainer = new THREE.Group()
      roverContainer.position.set(0, -50, 0)
      roverContainer.rotation.set(-Math.PI / 2, 0, 0)
      parent.add(roverContainer)
      roverContainer.add(robot)
      robot.updateMatrixWorld()

      // parse joints and initial positions
      robot.traverse((obj: THREE.Object3D) => {
        const joint = obj as THREE.Object3D & {
          jointType?: string
          jointValue?: number
          limit?: { lower: number; upper: number }
          setJointValue?: (val: number) => void
        }
        if (
          joint.jointType === 'revolute' ||
          joint.jointType === 'continuous' ||
          joint.jointType === 'prismatic'
        ) {
          const name = joint.name || 'unnamed_joint'
          const initialValue =
            typeof DEFAULT_JOINT_VALUES[name] === 'number'
              ? DEFAULT_JOINT_VALUES[name]
              : typeof joint.jointValue === 'number'
                ? joint.jointValue
                : 0
          joint.setJointValue?.(initialValue)
        }
      })

      // debug sliders
      robot.traverse((obj: THREE.Object3D) => {
        const joint = obj as THREE.Object3D & {
          jointType?: string
          limit?: { lower: number; upper: number }
          setJointValue?: (val: number) => void
        }
        if (
          joint.jointType === 'revolute' ||
          joint.jointType === 'continuous' ||
          joint.jointType === 'prismatic'
        ) {
          const name = joint.name || 'unnamed_joint'
          const min = joint.limit?.lower ?? -Math.PI
          const max = joint.limit?.upper ?? Math.PI
          const folder = gui.addFolder(name)
          const paramObj = { value: DEFAULT_JOINT_VALUES[name] ?? 0 }
          folder
            .add(paramObj, 'value', min, max, 0.01)
            .name(`${name} (${joint.jointType})`)
            .onChange((value: number) => joint.setJointValue?.(value))
        }
      })
    },
    undefined,
    (err: unknown) => console.error('Failed to load URDF:', err),
  )

  function updateJoints(joints: JointUpdate[]) {
    if (!rover) return
    rover.traverse((obj: THREE.Object3D) => {
      const joint = obj as THREE.Object3D & {
        jointType?: string
        setJointValue?: (val: number) => void
      }
      if (
        joint.jointType === 'revolute' ||
        joint.jointType === 'continuous' ||
        joint.jointType === 'prismatic'
      ) {
        const match = joints.find(j => j.name === joint.name)
        if (match) {
          joint.setJointValue?.(match.position)
        }
      }
    })
  }

  function updateIKTarget(position: Position3D | null) {
    if (!position) {
      ikSphere.visible = false
      return
    }
    ikSphere.position.set(position.x + 10, position.y, position.z)
    ikSphere.visible = true
  }

  return { updateJoints, updateIKTarget }
}

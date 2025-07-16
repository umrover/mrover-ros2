import * as THREE from 'three'
import GUI from 'lil-gui'
import URDFLoader from 'urdf-loader'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls'
// import { TrackballControls } from 'three/addons/controls/TrackballControls.js'

const defaultJointValues = {
  chassis_to_arm_a: 24.14,
  arm_a_to_arm_b: -0.785,
  arm_b_to_arm_c: 1.91,
  arm_c_to_arm_d: -1,
  arm_d_to_arm_e: -1.57,
  gripper_link: 0,
}

let rover = null;

export default function threeSetup() {
  // Canvas element
  const canvas = document.querySelector('canvas.webgl')
  // const gui = new GUI( { container: document.querySelector("canvas.webgl") } ); // ??
  const gui = new GUI()

  // Scene setup
  const scene = new THREE.Scene()
  scene.background = new THREE.Color(0x87ceeb)
  // scene.background = new THREE.Color(0xbbbbbb);

  // Lighting setup
  // Ambient Light (soft light)
  const ambientLight = new THREE.AmbientLight(0x808080, 1) // soft white light
  scene.add(ambientLight)

  // Directional Light (main light source)
  const directionalLight = new THREE.DirectionalLight(0xffffff, 1) // white light
  directionalLight.position.set(2, 3, 2) // Position of light source
  directionalLight.castShadow = true // Enable shadows
  scene.add(directionalLight)

  const axesHelper = new THREE.AxesHelper(50)
  scene.add(axesHelper)

  const manager = new THREE.LoadingManager()
  const loader = new URDFLoader(manager)
  loader.packages = {
    mrover: '/urdf',
  }

  // Load using absolute path from web root
  loader.load(
    // '/urdf/arm/arm.urdf',
    '/urdf/rover/rover.urdf',
    robot => {
      rover = robot
      robot.position.set(0, -50, 0)
      robot.rotation.x = -Math.PI / 2
      robot.updateMatrixWorld()
      scene.add(robot)

      // Add GUI controls for joint angles
      robot.traverse(obj => {
        if (
          obj.jointType === 'revolute' ||
          obj.jointType === 'continuous' ||
          obj.jointType === 'prismatic'
        ) {
          const name = obj.name || 'unnamed_joint'

          const initialValue =
            typeof defaultJointValues[name] === 'number'
              ? defaultJointValues[name]
              : typeof obj.jointValue === 'number'
                ? obj.jointValue
                : 0

          const min = obj.limit?.lower ?? -Math.PI
          const max = obj.limit?.upper ?? Math.PI

          const folder = gui.addFolder(name)
          const paramObj = { value: initialValue }

          obj.setJointValue(initialValue)

          folder
            .add(paramObj, 'value', min, max, 0.01)
            .name(`${name} (${obj.jointType})`)
            .onChange(value => {
              obj.setJointValue(value)
            })
        }
      })
    },
    undefined,
    err => {
      console.error('Failed to load URDF:', err)
    },
  )

  // Sizes for window resizing
  const sizes = {
    width: window.innerWidth,
    height: window.innerHeight,
  }

  // Resize event listenerreb
  window.addEventListener('resize', () => {
    sizes.width = window.innerWidth
    sizes.height = window.innerHeight
    // Update camera
    camera.aspect = sizes.width / sizes.height
    camera.updateProjectionMatrix()
    // Update renderer
    renderer.setSize(sizes.width, sizes.height)
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
  })

  // Camera setup
  const camera = new THREE.PerspectiveCamera(
    75,
    sizes.width / sizes.height,
    0.1,
    1000,
  )
  camera.position.x = 100
  camera.position.y = 50
  camera.position.z = 100
  camera.lookAt(0, 0, 0)
  scene.add(camera)

  // OrbitControls for the camera
  const controls = new OrbitControls(camera, canvas)
  controls.enableDamping = true

  // Renderer setup
  const renderer = new THREE.WebGLRenderer({
    antialias: true,
    canvas: canvas,
  })
  renderer.setSize(sizes.width, sizes.height)
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
  renderer.shadowMap.enabled = true
  renderer.shadowMap.type = THREE.PCFSoftShadowMap
  renderer.physicallyCorrectLights = true
  renderer.toneMapping = THREE.ACESFilmicToneMapping
  renderer.toneMappingExposure = 2.5

  // Animation loop (tick)
  // const clock = new THREE.Clock();

  const tick = () => {
    // const elapsedTime = clock.getElapsedTime();
    // Update controls and render
    // controls.update();
    renderer.render(scene, camera)
    window.requestAnimationFrame(tick)
  }

  tick() // Start the animation loop

  return () => {
    renderer.dispose() // Dispose renderer
  }
}

export function updatePose(joints) {
  if (!rover) return
  rover.traverse(obj => {
    if (
      obj.jointType === 'revolute' ||
      obj.jointType === 'continuous' ||
      obj.jointType === 'prismatic'
    ) {
      const joint = joints.find(j => j.name === obj.name)
      if (joint) {
        obj.setJointValue(joint.position)
      }
    }
  })
}

// export function fk(positions, scene, joints) {
//   let cumulativeMatrix = new THREE.Matrix4()
//   cumulativeMatrix.makeTranslation(new THREE.Vector3(0, 0, 0.439675)) // base_link offset

//   for (let i = 0; i < joints.length; ++i) {
//     let mesh = scene.getObjectByName(joints[i].name)
//     if (!mesh) continue

//     let localMatrix = new THREE.Matrix4()
//     let rotationAngle = positions[i]

//     if (joints[i].name === 'chassis') {
//       localMatrix.makeTranslation(0, rotationAngle, 0)
//     } else {
//       localMatrix.makeRotationY(rotationAngle)
//     }

//     let offset = new THREE.Vector3().fromArray(joints[i].translation)
//     localMatrix.setPosition(
//       new THREE.Vector3().setFromMatrixPosition(localMatrix).add(offset),
//     )

//     mesh.matrixAutoUpdate = false
//     mesh.matrix = cumulativeMatrix.clone()

//     cumulativeMatrix.multiply(localMatrix)
//   }
// }

// export function ik(target, targetCube) {
//   let quaternion = new THREE.Quaternion(...target.quaternion)
//   targetCube.position.set(...target.position)
//   targetCube.setRotationFromQuaternion(quaternion)
// }

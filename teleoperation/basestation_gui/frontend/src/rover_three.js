import * as THREE from 'three'
import GUI from 'lil-gui'
import URDFLoader from 'urdf-loader'
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js'
import { DRACOLoader } from 'three/addons/loaders/DRACOLoader.js'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls'

const defaultJointValues = {
  chassis_to_arm_a: 24.14,
  arm_a_to_arm_b: -0.785,
  arm_b_to_arm_c: 1.91,
  arm_c_to_arm_d: -1,
  arm_d_to_arm_e: -1.57,
  gripper_link: 0,
}

let rover = null
let ikTargetSphere = null
let current_camera_type = 'default'

// Costmap DataTexture
const costMapAnchor = new THREE.Object3D()
const costMapBlockWidth = 40
export const numCostMapBlocks = 40
const costMapSize = numCostMapBlocks * numCostMapBlocks
const costMapTextureData = new Uint8Array(costMapSize * 4)
const costMapTexture = new THREE.DataTexture(
  costMapTextureData,
  numCostMapBlocks,
  numCostMapBlocks,
  THREE.RGBAFormat,
)
costMapTexture.magFilter = THREE.NearestFilter
costMapTexture.minFilter = THREE.NearestFilter
const costMapSideLength = costMapBlockWidth * numCostMapBlocks
const costMapMaterial = new THREE.MeshBasicMaterial({ map: costMapTexture })
const costMapPlane = new THREE.Mesh(
  new THREE.PlaneGeometry(costMapSideLength, costMapSideLength),
  costMapMaterial,
)

// Costmap text overlay
const textCanvas = document.createElement('canvas')
const textCanvasSideLength = costMapBlockWidth * numCostMapBlocks
textCanvas.width = textCanvasSideLength
textCanvas.height = textCanvasSideLength
const textCanvasContext = textCanvas.getContext('2d')
const textCanvasTexture = new THREE.CanvasTexture(textCanvas)
const textCanvasMaterial = new THREE.MeshBasicMaterial({
  map: textCanvasTexture,
  transparent: true,
})
const textCanvasPlane = new THREE.Mesh(
  new THREE.PlaneGeometry(textCanvasSideLength, textCanvasSideLength),
  textCanvasMaterial,
)
textCanvasPlane.position.set(
  -costMapBlockWidth / 2,
  -49,
  -costMapBlockWidth / 2,
)

export default function threeSetup() {
  const canvas = document.querySelector('canvas.webgl')
  const gui = new GUI({ width: 400 })
  gui.hide()

  const scene = new THREE.Scene()
  scene.background = new THREE.Color(0x87ceeb)

  // Lighting
  const ambientLight = new THREE.AmbientLight(0x808080, 1)
  scene.add(ambientLight)

  const directionalLight = new THREE.DirectionalLight(0xffffff, 1)
  directionalLight.position.set(2, 3, 2)
  directionalLight.castShadow = true
  scene.add(directionalLight)

  // IK target sphere
  const sphereGeometry = new THREE.SphereGeometry(2, 16, 16)
  const sphereMaterial = new THREE.MeshStandardMaterial({
    color: 0xff0000,
    emissive: 0xff0000,
    emissiveIntensity: 0.3,
  })
  ikTargetSphere = new THREE.Mesh(sphereGeometry, sphereMaterial)
  ikTargetSphere.position.set(0, 0, 0)
  ikTargetSphere.visible = false
  scene.add(ikTargetSphere)

  // Costmap plane + text overlay
  scene.add(costMapAnchor)
  costMapPlane.position.set(-costMapBlockWidth / 2, -50, -costMapBlockWidth / 2)
  costMapPlane.rotation.x = -Math.PI / 2
  costMapAnchor.add(costMapPlane)

  const currentTextSize = costMapBlockWidth * 0.3
  textCanvasContext.fillStyle = 'white'
  textCanvasContext.font = currentTextSize + 'px Arial'
  textCanvasContext.textAlign = 'center'
  textCanvasContext.textBaseline = 'middle'
  textCanvasPlane.lookAt(-costMapBlockWidth / 2, 50, -costMapBlockWidth / 2)
  costMapAnchor.add(textCanvasPlane)

  // URDF loader
  const manager = new THREE.LoadingManager()
  const loader = new URDFLoader(manager)
  loader.packages = { mrover: '/urdf' }

  // Custom mesh loader for GLTF + Draco compressed meshes
  loader.loadMeshCb = function (path, manager, onComplete) {
    const gltfLoader = new GLTFLoader(manager)
    const dracoLoader = new DRACOLoader()
    dracoLoader.setDecoderPath('https://www.gstatic.com/draco/v1/decoders/')
    gltfLoader.setDRACOLoader(dracoLoader)
    gltfLoader.load(
      path,
      result => onComplete(result.scene),
      undefined,
      err => onComplete(null, err),
    )
  }

  loader.load(
    '/urdf/rover/rover.urdf',
    robot => {
      rover = robot

      // roverContainer: world position + ROS Z-up to Three.js Y-up frame conversion
      const roverContainer = new THREE.Group()
      roverContainer.position.set(0, -50, 0)
      roverContainer.rotation.x = -Math.PI / 2
      scene.add(roverContainer)

      // roverPivot: visual-only rotation so model faces forward (does not affect joint transforms)
      const roverPivot = new THREE.Group()
      roverPivot.rotation.z = Math.PI / 2
      roverContainer.add(roverPivot)

      roverPivot.add(robot)
      robot.updateMatrixWorld()

      // GUI controls for joint angles
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
            .onChange(value => obj.setJointValue(value))
        }
      })
    },
    undefined,
    err => console.error('Failed to load URDF:', err),
  )

  // Cameras
  const sizes = { width: canvas.clientWidth, height: canvas.clientHeight }
  const makeCamera = () =>
    new THREE.PerspectiveCamera(75, sizes.width / sizes.height, 0.1, 1000)

  const camera_types = {
    default: makeCamera(),
    follow: makeCamera(),
    'full arm': makeCamera(),
    arm: makeCamera(),
    'side arm': makeCamera(),
    top: makeCamera(),
  }

  camera_types['default'].position.set(100, 50, 100)
  camera_types['default'].lookAt(0, 0, 0)

  camera_types['follow'].position.set(-130, 120, 0)
  camera_types['follow'].lookAt(0, 30, 0)

  camera_types['full arm'].position.set(-20, 60, 0)
  camera_types['full arm'].lookAt(75, 0, 0)

  camera_types['arm'].position.set(0, 30, 10)
  camera_types['arm'].lookAt(75, 10, 10)

  camera_types['side arm'].position.set(25, 40, 70)
  camera_types['side arm'].lookAt(75, 0, 0)

  camera_types['top'].position.set(-1, 160, 0)
  camera_types['top'].lookAt(0, 0, 0)

  for (const cam of Object.values(camera_types)) {
    scene.add(cam)
  }

  // OrbitControls only attached to default camera
  const controls = new OrbitControls(camera_types['default'], canvas)

  // Renderer
  const renderer = new THREE.WebGLRenderer({ antialias: true, canvas })
  renderer.setSize(sizes.width, sizes.height)
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
  renderer.shadowMap.enabled = true
  renderer.shadowMap.type = THREE.PCFSoftShadowMap
  renderer.physicallyCorrectLights = true
  renderer.toneMapping = THREE.ACESFilmicToneMapping
  renderer.toneMappingExposure = 2.5

  const resizeObserver = new ResizeObserver(() => {
    sizes.width = canvas.clientWidth
    sizes.height = canvas.clientHeight
    for (const cam of Object.values(camera_types)) {
      cam.aspect = sizes.width / sizes.height
      cam.updateProjectionMatrix()
    }
    renderer.setSize(sizes.width, sizes.height)
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
  })
  resizeObserver.observe(canvas)

  const clock = new THREE.Clock()
  let timeSinceLastFrame = 0
  const frameInterval = 1 / 60

  const tick = () => {
    window.requestAnimationFrame(tick)
    const delta = clock.getDelta()
    timeSinceLastFrame += delta
    if (timeSinceLastFrame > frameInterval) {
      timeSinceLastFrame %= frameInterval
      controls.update()
      renderer.render(scene, camera_types[current_camera_type])
    }
  }
  tick()

  return () => {
    resizeObserver.disconnect()
    renderer.dispose()
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

export function updateIKTarget(position) {
  if (!ikTargetSphere) return
  if (
    position &&
    typeof position.x === 'number' &&
    typeof position.y === 'number' &&
    typeof position.z === 'number'
  ) {
    ikTargetSphere.position.set(position.x + 10, position.y, position.z)
    ikTargetSphere.visible = true
  } else if (position === null || position === undefined) {
    ikTargetSphere.visible = false
  }
}

export function set_camera_type(new_type) {
  current_camera_type = new_type
}

// Draws numeric cost values onto the text canvas (does not clear or mark for update)
function fillTextCanvas(gridData) {
  for (let i = 0; i < numCostMapBlocks; i++) {
    for (let j = 0; j < numCostMapBlocks; j++) {
      textCanvasContext.fillText(
        gridData[i * numCostMapBlocks + j],
        (j + 0.5) * costMapBlockWidth,
        (i + 0.5) * costMapBlockWidth,
      )
    }
  }
}

export function updateCostMapGrid(gridData) {
  for (let i = 0; i < costMapSize; i++) {
    const val = gridData[i]
    const idx = i * 4
    if (val === 0) {
      costMapTextureData[idx] = 0
      costMapTextureData[idx + 1] = 128
      costMapTextureData[idx + 2] = 0
    } else if (val < 0) {
      costMapTextureData[idx] = 0
      costMapTextureData[idx + 1] = 26
      costMapTextureData[idx + 2] = 0
    } else {
      costMapTextureData[idx] = Math.min(255, val * 2.55)
      costMapTextureData[idx + 1] = Math.max(0, 255 - val * 2.55)
      costMapTextureData[idx + 2] = 0
    }
    costMapTextureData[idx + 3] = 255
  }
  costMapTexture.needsUpdate = true

  textCanvasContext.clearRect(0, 0, textCanvas.width, textCanvas.height)
  fillTextCanvas(gridData)
  textCanvasTexture.needsUpdate = true
}

export function resetCostMapGrid() {
  for (let i = 0; i < costMapSize; i++) {
    const idx = i * 4
    costMapTextureData[idx] = 157
    costMapTextureData[idx + 1] = 0
    costMapTextureData[idx + 2] = 255
    costMapTextureData[idx + 3] = 255
  }
  costMapTexture.needsUpdate = true

  textCanvasContext.clearRect(0, 0, textCanvas.width, textCanvas.height)
  const tempGridData = new Array(costMapSize).fill(-100)
  fillTextCanvas(tempGridData)
  textCanvasTexture.needsUpdate = true
}

export function toggleCostMapGridVisibility() {
  costMapPlane.visible = !costMapPlane.visible
  textCanvasPlane.visible = !textCanvasPlane.visible
}

export function setCostMapRotation(rotationValue) {
  costMapAnchor.rotation.y = rotationValue
}

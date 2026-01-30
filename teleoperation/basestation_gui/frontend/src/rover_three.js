import * as THREE from 'three'
import GUI from 'lil-gui'
import URDFLoader from 'urdf-loader'
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
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

let current_camera_type = "default"

const costMapBlockWidth = 40
const numCostMapBlocks = 40
const costMapGridOffset = costMapBlockWidth * (numCostMapBlocks/2)
let costMapBlocks = []
const textCanvas = document.createElement('canvas');
const textCanvasSideLength = costMapBlockWidth * numCostMapBlocks
textCanvas.width = textCanvasSideLength;
textCanvas.height = textCanvasSideLength;
const textCanvasContext = textCanvas.getContext('2d');
const textCanvasTexture = new THREE.CanvasTexture(textCanvas);
const textCanvasMaterial = new THREE.MeshBasicMaterial({ map: textCanvasTexture, transparent: true });
const textCanvasPlane = new THREE.Mesh(new THREE.PlaneGeometry(textCanvasSideLength, textCanvasSideLength), textCanvasMaterial);
textCanvasPlane.position.x = -costMapBlockWidth / 2
textCanvasPlane.position.y = -49
textCanvasPlane.position.z = -costMapBlockWidth / 2

export default function threeSetup() {
  // Canvas element
  const canvas = document.querySelector('canvas.webgl')
  // const gui = new GUI( { container: document.querySelector("canvas.webgl") } ); // ??
  const gui = new GUI({ width: 400 })
  gui.hide()

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

  // const axesHelper = new THREE.AxesHelper(50)
  // scene.add(axesHelper)

  // Create IK target sphere
  const sphereGeometry = new THREE.SphereGeometry(2, 16, 16)
  const sphereMaterial = new THREE.MeshStandardMaterial({
    color: 0xff0000,
    emissive: 0xff0000,
    emissiveIntensity: 0.3,
  })
  ikTargetSphere = new THREE.Mesh(sphereGeometry, sphereMaterial)
  ikTargetSphere.position.set(0, 0, 0)
  ikTargetSphere.visible = false // Hidden by default until position is set
  scene.add(ikTargetSphere)

  //Create costmap grid  
  for(let i = 0, idx = 0; i < numCostMapBlocks; i++){
    for(let j = 0; j < numCostMapBlocks; j++, idx++){
      const currentGeometry = new THREE.BoxGeometry(costMapBlockWidth,1,costMapBlockWidth)
      const currentMaterial = new THREE.MeshStandardMaterial({
        color: 0x9d00ff,
      })
      
      costMapBlocks.push(new THREE.Mesh(currentGeometry, currentMaterial))

      const current_x = j * costMapBlockWidth - costMapGridOffset
      const current_z =  i * costMapBlockWidth - costMapGridOffset

      costMapBlocks[idx].position.x = current_x
      costMapBlocks[idx].position.y = -50
      costMapBlocks[idx].position.z = current_z

      scene.add(costMapBlocks[idx])
    }
  }

  //Initialize Text
  const currentTextSize = costMapBlockWidth * 0.3
  textCanvasContext.fillStyle = 'white';
  textCanvasContext.font = currentTextSize + 'px Arial';
  textCanvasContext.textAlign = 'center';
  textCanvasContext.textBaseline = 'middle';
  
  let tempGridData = []
  for(let i = 0; i < costMapBlockWidth * costMapBlockWidth; i++){
    tempGridData.push(-1)
  }

  fillTextCanvas(tempGridData)

  //Add textCanvasPlane to Scene
  textCanvasPlane.lookAt(-costMapBlockWidth / 2, 50, -costMapBlockWidth / 2)
  scene.add(textCanvasPlane);



  const manager = new THREE.LoadingManager()
  const loader = new URDFLoader(manager)
  loader.packages = {
    mrover: '/urdf',
  }

  loader.loadMeshCb = function(path, manager, onComplete) {
    const gltfLoader = new GLTFLoader(manager);
    const dracoLoader = new DRACOLoader();
    dracoLoader.setDecoderPath( 'https://www.gstatic.com/draco/v1/decoders/' );
    gltfLoader.setDRACOLoader( dracoLoader );
    gltfLoader.load(path, result => {
        onComplete(result.scene); // Pass the GLTF scene to urdf-loader
    }, undefined, err => {
        onComplete(null, err); // Handle errors
    });
  };

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

  const sizes = {
    width: canvas.clientWidth,
    height: canvas.clientHeight,
  }

  // Camera setup
  const camera_types = {
    "default": new THREE.PerspectiveCamera(
      75,
      sizes.width / sizes.height,
      0.1,
      1000,
    ),
    "follow": new THREE.PerspectiveCamera(
        75,
        sizes.width / sizes.height,
        0.1,
        1000,
    ),
    "full arm": new THREE.PerspectiveCamera(
        75,
        sizes.width / sizes.height,
        0.1,
        1000,
    ),
    "arm": new THREE.PerspectiveCamera(
        75,
        sizes.width / sizes.height,
        0.1,
        1000,
    ),
    "side arm": new THREE.PerspectiveCamera(
        75,
        sizes.width / sizes.height,
        0.1,
        1000,
    ),
    "top": new THREE.PerspectiveCamera(
        75,
        sizes.width / sizes.height,
        0.1,
        1000,
    )
  }

  camera_types["default"].position.x = 100
  camera_types["default"].position.y = 50
  camera_types["default"].position.z = 100
  camera_types["default"].lookAt(0, 0, 0)
  scene.add(camera_types["default"])

  camera_types["follow"].position.x = -130
  camera_types["follow"].position.y = 120
  camera_types["follow"].position.z = 0
  camera_types["follow"].lookAt(0, 30, 0)
  scene.add(camera_types["follow"])

  camera_types["full arm"].position.x = -20
  camera_types["full arm"].position.y = 60
  camera_types["full arm"].position.z = 0
  camera_types["full arm"].lookAt(75, 0, 0)
  scene.add(camera_types["arm"])

  camera_types["arm"].position.x = 0
  camera_types["arm"].position.y = 30
  camera_types["arm"].position.z = 10//15
  camera_types["arm"].lookAt(75, 10, 10)
  scene.add(camera_types["arm"])

  camera_types["side arm"].position.x = 25
  camera_types["side arm"].position.y = 40
  camera_types["side arm"].position.z = 70
  camera_types["side arm"].lookAt(75, 0, 0)
  scene.add(camera_types["side arm"])

  camera_types["top"].position.x = -1
  camera_types["top"].position.y = 160
  camera_types["top"].position.z = 0
  camera_types["top"].lookAt(0, 0, 0)
  scene.add(camera_types["top"])

  // OrbitControls for the camera
  const controls = new OrbitControls(camera_types["default"], canvas)
  //controls.target = (0, 0, 0)
  console.log("here -->")
  console.log(controls.target)

  // controls.enableDamping = true

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
  const fpsLimit = 60
  const frameInterval = 1 / fpsLimit

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

  // Update sphere position
  // Assuming position is an object with x, y, z properties
  if (
    position &&
    typeof position.x === 'number' &&
    typeof position.y === 'number' &&
    typeof position.z === 'number'
  ) {
    ikTargetSphere.position.set(position.x + 10, position.y, position.z)
    ikTargetSphere.visible = true
    console.log('pos ', position.x, position.y, position.z)
  } else if (position === null || position === undefined) {
    // Hide sphere if no position provided
    ikTargetSphere.visible = false
  }
}

export function set_camera_type(new_type) {
  current_camera_type = new_type
}

// Fills the textCanvas with values according to gridData
// Note that it does not clear the canvas, nor marks it for update
function fillTextCanvas(gridData){
  for(let i = 0; i < numCostMapBlocks; i++){
    for(let j = 0; j < numCostMapBlocks; j++){
      textCanvasContext.fillText(gridData[(i * numCostMapBlocks) + j], (j + 0.5) * costMapBlockWidth, (i + 0.5) * costMapBlockWidth)
    }
  }
}

export function updateCostMapGrid(gridData){
  for(let i = 0; i < numCostMapBlocks * numCostMapBlocks; i++){
    
    const newColor = new THREE.Color(gridData[i] * 0.01, 1 - gridData[i] * 0.01, 0)

    const newMaterial = new THREE.MeshStandardMaterial({
    color: newColor
    })

    costMapBlocks[i].material = newMaterial

    //costMapBlocks.material.color = THREE.Color(gridData[i] * 0.01, 1 - gridData[i] * 0.01, 0)
  }

  textCanvasContext.clearRect(0, 0, textCanvas.width, textCanvas.height);
  
  fillTextCanvas(gridData)
  textCanvasTexture.needsUpdate = true;
}

export function resetCostMapGrid(){
  for(let i = 0; i < numCostMapBlocks * numCostMapBlocks; i++){
    const newMaterial = new THREE.MeshStandardMaterial({
        color: 0x9d00ff,
      })
    costMapBlocks[i].material = newMaterial
  }

  textCanvasContext.clearRect(0, 0, textCanvas.width, textCanvas.height);
  
  let tempGridData = []
  for(let i = 0; i < costMapBlockWidth * costMapBlockWidth; i++){
    tempGridData.push(-100)
  }

  fillTextCanvas(tempGridData)
  textCanvasTexture.needsUpdate = true;
}

export function toggleCostMapGridVisibility(){
  for(let i = 0; i < numCostMapBlocks * numCostMapBlocks; i++){
    costMapBlocks[i].visible = !costMapBlocks[i].visible
  }

  textCanvasPlane.visible = !textCanvasPlane.visible
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

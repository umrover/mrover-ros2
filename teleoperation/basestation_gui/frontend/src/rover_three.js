import * as THREE from 'three'
import { FBXLoader } from 'three/examples/jsm/loaders/FBXLoader'
import { TrackballControls } from 'three/addons/controls/TrackballControls.js'

export default function threeSetup(containerId) {
  const container = document.getElementById(containerId)
  const canvas = {
    width: container.clientWidth,
    height: container.clientHeight
  }

  const scene = new THREE.Scene()
  const camera = new THREE.PerspectiveCamera(75, canvas.width / canvas.height, 0.1, 1000)
  camera.up.set(0, 0, 1)

  const directionalLight = new THREE.DirectionalLight(0xffffff, 0.5)
  directionalLight.position.set(1, 2, 3)
  scene.add(directionalLight)

  const renderer = new THREE.WebGLRenderer()
  renderer.setSize(canvas.width, canvas.height)
  container.appendChild(renderer.domElement)

  scene.background = new THREE.Color(0xeeeeee)

  let controls = new TrackballControls(camera, renderer.domElement)
  controls.rotateSpeed = 1.0
  controls.zoomSpeed = 1.2
  controls.panSpeed = 0.8
  controls.noZoom = false
  controls.noPan = false
  controls.staticMoving = true
  controls.dynamicDampingFactor = 0.3

  const joints = [
    { name: 'chassis', file: 'rover_chassis.fbx', translation: [0.164882, -0.200235, 0.051497], rotation: [0, 0, 0] },
    { name: 'a', file: 'arm_a.fbx', translation: [0.034346, 0, 0.049024], rotation: [0, 0, 0] },
    { name: 'b', file: 'arm_b.fbx', translation: [0.534365, 0, 0.009056], rotation: [0, 0, 0] },
    { name: 'c', file: 'arm_c.fbx', translation: [0.546033, 0, 0.088594], rotation: [0, 0, 0] },
    { name: 'd', file: 'arm_d.fbx', translation: [0.044886, 0, 0], rotation: [0, 0, 0] },
    { name: 'e', file: 'arm_e.fbx', translation: [0.044886, 0, 0], rotation: [0, 0, 0] }
  ]

  const loader = new FBXLoader()
  for (let i = 0; i < joints.length; ++i) {
    loader.load('/meshes/' + joints[i].file, (fbx) => {
      fbx.name = joints[i].name
      fbx.traverse((child) => {
        if (child.isMesh) {
          child.name = joints[i].name
          child.material = new THREE.MeshStandardMaterial({ color: (joints[i].name === 'd' || joints[i].name === 'e') ? 0x00ff00 : 0xffffff })
          child.scale.set(1, 1, 1)
          child.position.set(0, 0, 0)
          scene.add(child)
        }
      })
    })
  }

  camera.position.set(1.25, 1.25, 1.25)
  camera.lookAt(new THREE.Vector3(0, 0, 0))

  const targetCube = new THREE.Mesh(
    new THREE.BoxGeometry(0.1, 0.1, 0.1),
    new THREE.MeshBasicMaterial({ color: 0x00ff00 })
  )
  scene.add(targetCube)

  function animate() {
    requestAnimationFrame(animate)
    controls.update()
    renderer.render(scene, camera)
  }

  animate()

  // return {
  //   fk: (positions) => fk(positions, scene, joints),
  //   ik: (target) => ik(target, targetCube)
  // }
}

export function fk(positions, scene, joints) {
  let cumulativeMatrix = new THREE.Matrix4()
  cumulativeMatrix.makeTranslation(new THREE.Vector3(0, 0, 0.439675)) // base_link offset

  for (let i = 0; i < joints.length; ++i) {
    let mesh = scene.getObjectByName(joints[i].name)
    if (!mesh) continue

    let localMatrix = new THREE.Matrix4()
    let rotationAngle = positions[i]

    if (joints[i].name === 'chassis') {
      localMatrix.makeTranslation(0, rotationAngle, 0)
    } else {
      localMatrix.makeRotationY(rotationAngle)
    }

    let offset = new THREE.Vector3().fromArray(joints[i].translation)
    localMatrix.setPosition(
      new THREE.Vector3().setFromMatrixPosition(localMatrix).add(offset)
    )

    mesh.matrixAutoUpdate = false
    mesh.matrix = cumulativeMatrix.clone()

    cumulativeMatrix.multiply(localMatrix)
  }
}

export function ik(target, targetCube) {
  let quaternion = new THREE.Quaternion(...target.quaternion)
  targetCube.position.set(...target.position)
  targetCube.setRotationFromQuaternion(quaternion)
}
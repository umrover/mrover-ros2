import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'

export type CameraType = 'default' | 'follow' | 'full arm' | 'arm' | 'side arm' | 'top'

type CameraMap = { [K in CameraType]: THREE.PerspectiveCamera }

export interface CameraManager {
  cameras: CameraMap
  controls: OrbitControls
  setType: (type: CameraType) => void
  getActive: () => THREE.PerspectiveCamera
  updateAspect: (width: number, height: number) => void
}

export function createCameras(
  canvas: HTMLCanvasElement,
  scene: THREE.Scene,
): CameraManager {
  const aspect = canvas.clientWidth / canvas.clientHeight
  const makeCamera = () => new THREE.PerspectiveCamera(75, aspect, 0.1, 1000)

  const cameras: CameraMap = {
    default: makeCamera(),
    follow: makeCamera(),
    'full arm': makeCamera(),
    arm: makeCamera(),
    'side arm': makeCamera(),
    top: makeCamera(),
  }

  cameras.default.position.set(100, 50, -100)
  cameras.default.lookAt(0, 0, 0)

  cameras.follow.position.set(-130, 120, 0)
  cameras.follow.lookAt(0, 30, 0)

  cameras['full arm'].position.set(-20, 60, 0)
  cameras['full arm'].lookAt(75, 0, 0)

  cameras.arm.position.set(0, 30, 10)
  cameras.arm.lookAt(75, 10, 10)

  cameras['side arm'].position.set(25, 40, 70)
  cameras['side arm'].lookAt(75, 0, 0)

  cameras.top.position.set(-1, 160, 0)
  cameras.top.lookAt(0, 0, 0)

  for (const cam of Object.values(cameras)) {
    scene.add(cam)
  }

  // OrbitControls only attached to default camera
  const controls = new OrbitControls(cameras.default, canvas)

  let activeCamera = cameras.default

  function setType(type: CameraType) {
    activeCamera = cameras[type]
  }

  function getActive(): THREE.PerspectiveCamera {
    return activeCamera
  }

  function updateAspect(width: number, height: number) {
    for (const cam of Object.values(cameras)) {
      cam.aspect = width / height
      cam.updateProjectionMatrix()
    }
  }

  return { cameras, controls, setType, getActive, updateAspect }
}

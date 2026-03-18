import * as THREE from 'three'
import { OrbitControls } from 'three/addons/controls/OrbitControls.js'

export type CameraType = 'orbit' | 'follow' | 'side arm' | 'top'

type CameraMap = { [K in CameraType]: THREE.PerspectiveCamera }

interface CameraPreset {
  position: THREE.Vector3
  target: THREE.Vector3
}

export interface CameraManager {
  cameras: CameraMap
  controls: OrbitControls
  setType: (type: CameraType) => void
  getActive: () => THREE.PerspectiveCamera
  resetActive: () => void
  setNavAzimuth: (radians: number) => void
  tickNav: () => void
  updateAspect: (width: number, height: number) => void
}

const PRESETS: Record<CameraType, CameraPreset> = {
  orbit: { position: new THREE.Vector3(100, 50, -100), target: new THREE.Vector3(0, 0, 0) },
  follow: { position: new THREE.Vector3(-130, 120, 0), target: new THREE.Vector3(0, 30, 0) },
  'side arm': { position: new THREE.Vector3(25, 40, 70), target: new THREE.Vector3(75, 0, 0) },
  top: { position: new THREE.Vector3(0, 300, 0), target: new THREE.Vector3(0, 0, 0) },
}

export function createCameras(
  canvas: HTMLCanvasElement,
  scene: THREE.Scene,
): CameraManager {
  const aspect = canvas.clientWidth / canvas.clientHeight
  const makeCamera = () => new THREE.PerspectiveCamera(75, aspect, 0.1, 1000)

  const cameras: CameraMap = {
    orbit: makeCamera(),
    follow: makeCamera(),
    'side arm': makeCamera(),
    top: makeCamera(),
  }

  for (const [key, cam] of Object.entries(cameras)) {
    const preset = PRESETS[key as CameraType]
    cam.position.copy(preset.position)
    cam.lookAt(preset.target)
    scene.add(cam)
  }

  const controls = new OrbitControls(cameras.orbit, canvas)

  let activeType: CameraType = 'orbit'
  let navAzimuth = 0
  let navActive = false

  function setType(type: CameraType) {
    const wasOrbitOrTop = activeType === 'orbit' || activeType === 'top'
    activeType = type

    if (type === 'orbit') {
      navActive = false
      controls.object = cameras.orbit
      controls.target.copy(PRESETS.orbit.target)
      controls.enableRotate = true
      controls.enablePan = true
      controls.enabled = true
      controls.update()
    } else if (type === 'top') {
      navActive = true
      navAzimuth = 0
      controls.object = cameras.top
      controls.target.set(0, 0, 0)
      controls.enableRotate = false
      controls.enablePan = false
      controls.enabled = true
      controls.update()
      applyNavOrientation()
    } else if (wasOrbitOrTop) {
      navActive = false
      controls.enabled = false
    }
  }

  function getActive(): THREE.PerspectiveCamera {
    return cameras[activeType]
  }

  function resetActive() {
    const preset = PRESETS[activeType]
    const cam = cameras[activeType]
    cam.position.copy(preset.position)
    if (activeType === 'orbit') {
      cam.up.set(0, 1, 0)
      cam.lookAt(preset.target)
      controls.target.copy(preset.target)
      controls.update()
    } else if (activeType === 'top') {
      controls.target.set(0, 0, 0)
      controls.update()
      navAzimuth = 0
      applyNavOrientation()
    }
  }

  function setNavAzimuth(radians: number) {
    navAzimuth = radians
    if (navActive) {
      applyNavOrientation()
    }
  }

  function applyNavOrientation() {
    const cam = cameras.top
    cam.up.set(Math.sin(navAzimuth), 0, Math.cos(navAzimuth))
    cam.lookAt(controls.target)
  }

  // Call after controls.update() each frame to re-apply nav orientation
  // (OrbitControls caches the initial up vector quaternion and overrides ours)
  function tickNav() {
    if (navActive) {
      applyNavOrientation()
    }
  }

  function updateAspect(width: number, height: number) {
    for (const cam of Object.values(cameras)) {
      cam.aspect = width / height
      cam.updateProjectionMatrix()
    }
  }

  return { cameras, controls, setType, getActive, resetActive, setNavAzimuth, tickNav, updateAspect }
}

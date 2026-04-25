<<<<<<< HEAD
=======
import * as THREE from 'three'
>>>>>>> origin/main
import { ref } from 'vue'
import { createScene, type SceneContext } from '@/components/three/scene'
import { createCameras, type CameraManager, CameraType } from '@/components/three/cameras'
import { createCostmap, NUM_COSTMAP_BLOCKS, type CostmapRenderer } from '@/components/three/costmap'
<<<<<<< HEAD
import { loadRover, type RoverModel, type JointUpdate, type Position3D } from '@/components/three/rover-model'

export type { JointUpdate, Position3D }
=======
import { loadRover, type RoverModel, type JointUpdate } from '@/components/three/rover-model'

export type { JointUpdate }
>>>>>>> origin/main
export { CameraType, NUM_COSTMAP_BLOCKS }

export function useRoverScene() {
  let sceneCtx: SceneContext | null = null
  let cameraManager: CameraManager | null = null
  let costmap: CostmapRenderer | null = null
  let roverModel: RoverModel | null = null
<<<<<<< HEAD
=======
  let roverPivot: THREE.Group | null = null
>>>>>>> origin/main

  const cameraType = ref<CameraType>(CameraType.Orbit)

  function setup(canvas: HTMLCanvasElement) {
    sceneCtx = createScene(canvas)
<<<<<<< HEAD
    cameraManager = createCameras(canvas, sceneCtx.scene)
    costmap = createCostmap(sceneCtx.scene)
    roverModel = loadRover(sceneCtx.scene)
=======

    roverPivot = new THREE.Group()
    sceneCtx.scene.add(roverPivot)

    cameraManager = createCameras(canvas, sceneCtx.scene, roverPivot)
    costmap = createCostmap(sceneCtx.scene)
    roverModel = loadRover(roverPivot)
>>>>>>> origin/main

    const resizeObserver = new ResizeObserver(() => {
      cameraManager?.updateAspect(canvas.clientWidth, canvas.clientHeight)
    })
    resizeObserver.observe(canvas)

    sceneCtx.startRenderLoop(
      () => cameraManager!.getActive(),
      () => {
        cameraManager?.controls.update()
        cameraManager?.tickNav()
      },
    )
  }

  function dispose() {
    sceneCtx?.dispose()
    sceneCtx = null
    cameraManager = null
    costmap = null
    roverModel = null
<<<<<<< HEAD
=======
    roverPivot = null
>>>>>>> origin/main
  }

  function setCamera(type: CameraType) {
    cameraManager?.setType(type)
    cameraType.value = type
  }

  function updateCostMap(gridData: number[]) {
    costmap?.update(gridData)
  }

  function resetCostMap() {
    costmap?.reset()
  }

  function toggleCostMapVisibility() {
    costmap?.toggleVisibility()
  }

  function setCostMapVisibility(visible: boolean) {
    costmap?.setVisibility(visible)
  }

<<<<<<< HEAD
  function setCostMapRotation(radians: number) {
    costmap?.setRotation(radians)
  }

=======
>>>>>>> origin/main
  function resetCamera() {
    cameraManager?.resetActive()
  }

  function setNavAzimuth(radians: number) {
    cameraManager?.setNavAzimuth(radians)
  }

  function updateJoints(joints: JointUpdate[]) {
    roverModel?.updateJoints(joints)
  }

<<<<<<< HEAD
  function updateIKTarget(position: Position3D | null) {
    roverModel?.updateIKTarget(position)
  }

  function setRoverHeading(radians: number) {
    roverModel?.setHeading(radians)
=======
  function setRoverHeading(radians: number) {
    if (roverPivot) {
      roverPivot.rotation.y = radians
    }
>>>>>>> origin/main
  }

  return {
    setup,
    dispose,
    cameraType,
    setCamera,
    resetCamera,
    setNavAzimuth,
    updateCostMap,
    resetCostMap,
    toggleCostMapVisibility,
    setCostMapVisibility,
<<<<<<< HEAD
    setCostMapRotation,
    updateJoints,
    updateIKTarget,
=======
    updateJoints,
>>>>>>> origin/main
    setRoverHeading,
    NUM_COSTMAP_BLOCKS,
  }
}

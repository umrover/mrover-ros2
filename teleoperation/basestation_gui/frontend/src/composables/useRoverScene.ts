import * as THREE from 'three'
import { ref } from 'vue'
import { createScene, type SceneContext } from '@/components/three/scene'
import { createCameras, type CameraManager, CameraType } from '@/components/three/cameras'
import { createCostmap, NUM_COSTMAP_BLOCKS, type CostmapRenderer } from '@/components/three/costmap'
import { loadRover, type RoverModel, type JointUpdate, type Position3D } from '@/components/three/rover-model'

export type { JointUpdate, Position3D }
export { CameraType, NUM_COSTMAP_BLOCKS }

export function useRoverScene() {
  let sceneCtx: SceneContext | null = null
  let cameraManager: CameraManager | null = null
  let costmap: CostmapRenderer | null = null
  let roverModel: RoverModel | null = null
  let roverPivot: THREE.Group | null = null

  const cameraType = ref<CameraType>(CameraType.Orbit)

  function setup(canvas: HTMLCanvasElement) {
    sceneCtx = createScene(canvas)

    roverPivot = new THREE.Group()
    sceneCtx.scene.add(roverPivot)

    cameraManager = createCameras(canvas, sceneCtx.scene, roverPivot)
    costmap = createCostmap(sceneCtx.scene)
    roverModel = loadRover(roverPivot)

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
    roverPivot = null
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

  function resetCamera() {
    cameraManager?.resetActive()
  }

  function setNavAzimuth(radians: number) {
    cameraManager?.setNavAzimuth(radians)
  }

  function updateJoints(joints: JointUpdate[]) {
    roverModel?.updateJoints(joints)
  }

  function updateIKTarget(position: Position3D | null) {
    roverModel?.updateIKTarget(position)
  }

  function setRoverHeading(radians: number) {
    if (roverPivot) {
      roverPivot.rotation.y = radians
    }
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
    updateJoints,
    updateIKTarget,
    setRoverHeading,
    NUM_COSTMAP_BLOCKS,
  }
}

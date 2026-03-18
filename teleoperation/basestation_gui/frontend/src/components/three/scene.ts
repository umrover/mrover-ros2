import * as THREE from 'three'

export interface SceneContext {
  scene: THREE.Scene
  renderer: THREE.WebGLRenderer
  startRenderLoop: (getActiveCamera: () => THREE.Camera, onUpdate?: () => void) => void
  dispose: () => void
}

export function createScene(canvas: HTMLCanvasElement): SceneContext {
  const scene = new THREE.Scene()
  scene.background = new THREE.Color(0x87ceeb)

  // Lighting
  const ambientLight = new THREE.AmbientLight(0x808080, 1)
  scene.add(ambientLight)

  const directionalLight = new THREE.DirectionalLight(0xffffff, 1)
  directionalLight.position.set(2, 3, 2)
  directionalLight.castShadow = true
  scene.add(directionalLight)

  // Renderer
  const renderer = new THREE.WebGLRenderer({ antialias: true, canvas })
  renderer.setSize(canvas.clientWidth, canvas.clientHeight, false)
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
  renderer.shadowMap.enabled = true
  renderer.shadowMap.type = THREE.PCFSoftShadowMap
  renderer.toneMapping = THREE.ACESFilmicToneMapping
  renderer.toneMappingExposure = 2.5

  // Resize handling - observe parent to avoid feedback loop with setSize
  const resizeObserver = new ResizeObserver(() => {
    renderer.setSize(canvas.clientWidth, canvas.clientHeight, false)
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))
  })
  resizeObserver.observe(canvas.parentElement!)

  let animationFrameId = 0

  function startRenderLoop(
    getActiveCamera: () => THREE.Camera,
    onUpdate?: () => void,
  ) {
    let lastTime = performance.now()
    let timeSinceLastFrame = 0
    const frameInterval = 1 / 60

    const tick = () => {
      animationFrameId = window.requestAnimationFrame(tick)
      const now = performance.now()
      const delta = (now - lastTime) / 1000
      lastTime = now
      timeSinceLastFrame += delta
      if (timeSinceLastFrame > frameInterval) {
        timeSinceLastFrame %= frameInterval
        onUpdate?.()
        renderer.render(scene, getActiveCamera())
      }
    }
    tick()
  }

  function dispose() {
    if (animationFrameId) {
      window.cancelAnimationFrame(animationFrameId)
    }
    resizeObserver.disconnect()
    renderer.dispose()
  }

  return { scene, renderer, startRenderLoop, dispose }
}

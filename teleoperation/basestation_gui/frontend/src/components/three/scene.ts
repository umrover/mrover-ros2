import * as THREE from 'three'

export interface SceneContext {
  scene: THREE.Scene
  renderer: THREE.WebGLRenderer
  markDirty: () => void
  startRenderLoop: (getActiveCamera: () => THREE.Camera, onUpdate?: () => void) => void
  dispose: () => void
}

export function createScene(canvas: HTMLCanvasElement): SceneContext {
  const scene = new THREE.Scene()
  scene.background = new THREE.Color(0x87ceeb)

  const ambientLight = new THREE.AmbientLight(0x808080, 1)
  scene.add(ambientLight)

  const directionalLight = new THREE.DirectionalLight(0xffffff, 1)
  directionalLight.position.set(2, 3, 2)
  directionalLight.castShadow = false
  scene.add(directionalLight)

  const renderer = new THREE.WebGLRenderer({ antialias: false, canvas, powerPreference: 'low-power' })
  renderer.setSize(canvas.clientWidth, canvas.clientHeight, false)
  renderer.setPixelRatio(1)
  renderer.shadowMap.enabled = false

  let needsRender = true

  function markDirty() {
    needsRender = true
  }

  const resizeObserver = new ResizeObserver(() => {
    renderer.setSize(canvas.clientWidth, canvas.clientHeight, false)
    markDirty()
  })
  resizeObserver.observe(canvas.parentElement!)

  let animationFrameId = 0
  let lastRenderTime = 0
  const MIN_RENDER_INTERVAL_MS = 100 // 10 FPS cap

  function startRenderLoop(
    getActiveCamera: () => THREE.Camera,
    onUpdate?: () => void,
  ) {
    const tick = (now: DOMHighResTimeStamp) => {
      animationFrameId = window.requestAnimationFrame(tick)

      if (document.hidden) return
      if (!needsRender) return
      if (now - lastRenderTime < MIN_RENDER_INTERVAL_MS) return

      needsRender = false
      lastRenderTime = now
      onUpdate?.()
      renderer.render(scene, getActiveCamera())
    }
    animationFrameId = window.requestAnimationFrame(tick)

    // Resume after the tab becomes visible again
    document.addEventListener('visibilitychange', () => {
      if (!document.hidden && needsRender) {
        markDirty()
      }
    })
  }

  function dispose() {
    if (animationFrameId) {
      window.cancelAnimationFrame(animationFrameId)
    }
    resizeObserver.disconnect()
    renderer.dispose()
  }

  return { scene, renderer, markDirty, startRenderLoop, dispose }
}

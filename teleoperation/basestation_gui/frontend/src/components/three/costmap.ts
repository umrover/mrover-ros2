import * as THREE from 'three'

export const NUM_COSTMAP_BLOCKS = 40

const BLOCK_WIDTH = 40
const GRID_SIZE = NUM_COSTMAP_BLOCKS * NUM_COSTMAP_BLOCKS
const SIDE_LENGTH = BLOCK_WIDTH * NUM_COSTMAP_BLOCKS

export interface CostmapRenderer {
  update: (gridData: number[]) => void
  reset: () => void
  toggleVisibility: () => void
  setVisibility: (visible: boolean) => void
  setRotation: (radians: number) => void
}

export function createCostmap(scene: THREE.Scene): CostmapRenderer {
  const anchor = new THREE.Object3D()
  scene.add(anchor)

  // Color plane via DataTexture
  const textureData = new Uint8Array(GRID_SIZE * 4)
  const texture = new THREE.DataTexture(
    textureData,
    NUM_COSTMAP_BLOCKS,
    NUM_COSTMAP_BLOCKS,
    THREE.RGBAFormat,
  )
  texture.magFilter = THREE.NearestFilter
  texture.minFilter = THREE.NearestFilter
  texture.flipY = true

  const colorPlane = new THREE.Mesh(
    new THREE.PlaneGeometry(SIDE_LENGTH, SIDE_LENGTH),
    new THREE.MeshBasicMaterial({ map: texture }),
  )
  colorPlane.position.set(-BLOCK_WIDTH / 2, -50, -BLOCK_WIDTH / 2)
  colorPlane.rotation.x = -Math.PI / 2
  anchor.add(colorPlane)

  // Text overlay via CanvasTexture
  const textCanvas = document.createElement('canvas')
  textCanvas.width = SIDE_LENGTH
  textCanvas.height = SIDE_LENGTH
  const ctx = textCanvas.getContext('2d')!
  const textTexture = new THREE.CanvasTexture(textCanvas)

  const textPlane = new THREE.Mesh(
    new THREE.PlaneGeometry(SIDE_LENGTH, SIDE_LENGTH),
    new THREE.MeshBasicMaterial({ map: textTexture, transparent: true }),
  )
  textPlane.position.set(-BLOCK_WIDTH / 2, -49, -BLOCK_WIDTH / 2)
  textPlane.lookAt(-BLOCK_WIDTH / 2, 50, -BLOCK_WIDTH / 2)
  anchor.add(textPlane)

  // Initialize text canvas style
  const fontSize = BLOCK_WIDTH * 0.3
  ctx.fillStyle = 'white'
  ctx.font = fontSize + 'px Arial'
  ctx.textAlign = 'center'
  ctx.textBaseline = 'middle'

  function fillText(gridData: number[]) {
    for (let i = 0; i < NUM_COSTMAP_BLOCKS; i++) {
      for (let j = 0; j < NUM_COSTMAP_BLOCKS; j++) {
        ctx.fillText(
          String(gridData[i * NUM_COSTMAP_BLOCKS + j]),
          (j + 0.5) * BLOCK_WIDTH,
          (i + 0.5) * BLOCK_WIDTH,
        )
      }
    }
  }

  function update(gridData: number[]) {
    for (let i = 0; i < GRID_SIZE; i++) {
      const val = gridData[i] ?? -1
      const idx = i * 4
      if (val === 0) {
        textureData[idx] = 0
        textureData[idx + 1] = 128
        textureData[idx + 2] = 0
      } else if (val < 0) {
        textureData[idx] = 0
        textureData[idx + 1] = 26
        textureData[idx + 2] = 0
      } else {
        textureData[idx] = Math.min(255, val * 2.55)
        textureData[idx + 1] = Math.max(0, 255 - val * 2.55)
        textureData[idx + 2] = 0
      }
      textureData[idx + 3] = 255
    }
    texture.needsUpdate = true

    ctx.clearRect(0, 0, SIDE_LENGTH, SIDE_LENGTH)
    fillText(gridData)
    textTexture.needsUpdate = true
  }

  function reset() {
    for (let i = 0; i < GRID_SIZE; i++) {
      const idx = i * 4
      textureData[idx] = 157
      textureData[idx + 1] = 0
      textureData[idx + 2] = 255
      textureData[idx + 3] = 255
    }
    texture.needsUpdate = true

    ctx.clearRect(0, 0, SIDE_LENGTH, SIDE_LENGTH)
    fillText(new Array(GRID_SIZE).fill(-100))
    textTexture.needsUpdate = true
  }

  function toggleVisibility() {
    colorPlane.visible = !colorPlane.visible
    textPlane.visible = !textPlane.visible
  }

  function setVisibility(visible: boolean) {
    colorPlane.visible = visible
    textPlane.visible = visible
  }

  function setRotation(radians: number) {
    anchor.rotation.y = radians
  }

  return { update, reset, toggleVisibility, setVisibility, setRotation }
}

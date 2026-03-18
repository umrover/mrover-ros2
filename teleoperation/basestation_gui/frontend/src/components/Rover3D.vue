<template>
  <div class="rover3d-root">
    <canvas class="webgl"></canvas>
    <div class="rover3d-toolbar">
      <div class="relative">
        <button
          type="button"
          class="toolbar-btn"
          @click="cameraDropdownOpen = !cameraDropdownOpen">
          {{ cameraLabels[camera_type] }}
          <i class="bi bi-chevron-down"></i>
        </button>
        <ul class="cmd-dropdown-menu left-0 right-auto" :class="{ show: cameraDropdownOpen }">
          <li v-for="(label, key) in cameraLabels" :key="key">
            <button
              class="cmd-dropdown-item"
              :class="{ active: camera_type === key }"
              @click="setCamera(key); cameraDropdownOpen = false">
              {{ label }}
            </button>
          </li>
        </ul>
      </div>

      <div class="flex gap-1">
        <button
          type="button"
          class="toolbar-btn"
          :class="{ 'toolbar-btn-active': costmapVisible }"
          @click="toggleCostMap()">
          Cost Map
        </button>
        <button
          type="button"
          class="toolbar-btn"
          :class="{ 'toolbar-btn-active': doCostmapRotation }"
          @click="toggleCostmapRotation()">
          Rotation
        </button>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, onMounted, onBeforeUnmount } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import type { ControllerStateMessage, IkFeedbackMessage, OccupancyGridMessage } from '@/types/websocket'
import type { OrientationMessage } from '@/types/coordinates'
import { quaternionToMapAngle } from '@/utils/map'
import { useRoverScene, NUM_COSTMAP_BLOCKS, type CameraType } from '@/composables/useRoverScene'

const { onMessage, setupWebSocket, closeWebSocket } = useWebsocketStore()

const {
  setup: setupScene,
  dispose: disposeScene,
  cameraType: camera_type,
  setCamera,
  updateCostMap,
  toggleCostMapVisibility,
  setCostMapVisibility,
  setCostMapRotation,
  updateJoints,
  updateIKTarget,
} = useRoverScene()

const cameraDropdownOpen = ref(false)
const cameraLabels: Record<CameraType, string> = {
  default: 'Default',
  follow: 'Follow',
  arm: 'Arm',
  'full arm': 'Full Arm',
  'side arm': 'Side Arm',
  top: 'Top Down',
}

const jointNameMap: Record<string, string> = {
  joint_a: 'chassis_to_arm_a',
  joint_b: 'arm_a_to_arm_b',
  joint_c: 'arm_b_to_arm_c',
  joint_de_pitch: 'arm_c_to_arm_d',
  joint_de_roll: 'arm_d_to_arm_e',
  gripper: 'gripper_link',
}

const COSTMAP_VISIBLE_KEY = 'rover3d.costmapVisible'
const costmapVisible = ref(localStorage.getItem(COSTMAP_VISIBLE_KEY) !== 'false')

function toggleCostMap() {
  costmapVisible.value = !costmapVisible.value
  localStorage.setItem(COSTMAP_VISIBLE_KEY, String(costmapVisible.value))
  toggleCostMapVisibility()
}

onMounted(() => {
  const canvas = document.querySelector('canvas.webgl') as HTMLCanvasElement
  setupScene(canvas)
  if (!costmapVisible.value) {
    setCostMapVisibility(false)
  }
  setupWebSocket('nav')
})

onBeforeUnmount(() => {
  disposeScene()
  closeWebSocket('nav')
})

onMessage<ControllerStateMessage>('arm', 'arm_state', (msg) => {
  const joints = msg.names.map((name: string, index: number) => {
    const urdfName = jointNameMap[name] || name
    const rawPosition: number = msg.positions[index] ?? 0
    const position = urdfName === 'chassis_to_arm_a'
      ? rawPosition * -100 + 40
      : rawPosition

    return { name: urdfName, position }
  })

  updateJoints(joints)
})

onMessage<IkFeedbackMessage>('arm', 'ik_feedback', (msg) => {
  updateIKTarget({
    x: msg.pos.x * 100,
    y: msg.pos.z * 100,
    z: msg.pos.y * -100 + 20,
  })
})

let roverMapPos = { x: 0, y: 0 }
const rover_bearing_deg = ref(0)
const doCostmapRotation = ref(true)

onMessage<OccupancyGridMessage>('drive', 'costmap', (msg) => {
  const { resolution, width, height, origin } = msg.info

  const roverCol = Math.floor((roverMapPos.x - origin.position.x) / resolution)
  const roverRow = Math.floor((roverMapPos.y - origin.position.y) / resolution)

  const halfBlocks = Math.floor(NUM_COSTMAP_BLOCKS / 2)
  const startCol = roverCol - halfBlocks
  const startRow = roverRow - halfBlocks

  const processedData: number[] = []
  for (let i = NUM_COSTMAP_BLOCKS - 1, k = 0; i >= 0; --i) {
    for (let j = 0; j < NUM_COSTMAP_BLOCKS; ++j, ++k) {
      const col = startCol + j
      const row = startRow + i
      if (col < 0 || col >= width || row < 0 || row >= height) {
        processedData[k] = -1
      } else {
        processedData[k] = msg.data[row * width + col] ?? -1
      }
    }
  }

  updateCostMap(processedData)

  if (doCostmapRotation.value) {
    setCostMapRotation(Math.PI * rover_bearing_deg.value / 180 - Math.PI / 2)
  }
})

onMessage<OrientationMessage>('nav', 'orientation', (msg) => {
  rover_bearing_deg.value = quaternionToMapAngle(msg.orientation)
  if (msg.position) {
    roverMapPos = { x: msg.position.x, y: msg.position.y }
  }
})

function toggleCostmapRotation() {
  if (doCostmapRotation.value) {
    doCostmapRotation.value = false
    setCostMapRotation(0)
  } else {
    doCostmapRotation.value = true
  }
}
</script>

<style scoped>
.rover3d-root {
  position: relative;
  width: 100%;
  height: 100%;
}

.webgl {
  display: block;
  width: 100%;
  height: 100%;
}

.rover3d-toolbar {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0.5rem;
  pointer-events: none;
}

.rover3d-toolbar > * {
  pointer-events: auto;
}

.toolbar-btn {
  display: inline-flex;
  align-items: center;
  gap: 0.25rem;
  padding: 0.25rem 0.5rem;
  font-size: 0.75rem;
  font-weight: 500;
  font-family: var(--cmd-font-mono);
  color: var(--text-primary);
  background: color-mix(in srgb, var(--card-bg) 80%, transparent);
  backdrop-filter: blur(8px);
  border: var(--cmd-border-width) solid var(--cmd-panel-border);
  border-radius: var(--cmd-radius-md);
  cursor: pointer;
  transition: all var(--cmd-transition);
}

.toolbar-btn:hover {
  background: var(--card-bg);
  border-color: var(--control-primary);
}

.toolbar-btn-active {
  color: #fff;
  background: color-mix(in srgb, var(--control-primary) 85%, transparent);
  border-color: var(--control-primary);
}

.toolbar-btn-active:hover {
  background: var(--control-primary);
}
</style>

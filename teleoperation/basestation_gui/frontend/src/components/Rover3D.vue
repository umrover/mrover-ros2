<template>
  <div class="rover3d-root">
    <canvas
      class="webgl"
      @click="closeDropdowns()"
      @pointerdown="onPointerDown"
      @pointermove="onPointerMove"
      @pointerup="onPointerUp"
    ></canvas>
    <div class="overlay-toolbar left-0 right-0 items-center justify-between">
      <div class="flex gap-1">
        <div class="relative">
          <button
            type="button"
            class="overlay-toolbar-btn"
            @click="viewDropdownOpen = !viewDropdownOpen">
            {{ viewLabels[viewMode] }}
            <i class="bi bi-chevron-down"></i>
          </button>
          <ul class="dropdown-menu left-0 right-auto" :class="{ show: viewDropdownOpen }">
            <li v-for="(label, key) in viewLabels" :key="key">
              <button
                class="dropdown-item"
                :class="{ active: viewMode === key }"
                @click="switchView(key); viewDropdownOpen = false">
                {{ label }}
              </button>
            </li>
          </ul>
        </div>

        <template v-if="isTopMode">
          <div class="relative">
            <button
              type="button"
              class="overlay-toolbar-btn"
              @click="rotationDropdownOpen = !rotationDropdownOpen">
              {{ rotationLabels[rotationMode] }}
              <i class="bi bi-chevron-down"></i>
            </button>
            <ul class="dropdown-menu left-0 right-auto" :class="{ show: rotationDropdownOpen }">
              <li v-for="(label, key) in rotationLabels" :key="key">
                <button
                  class="dropdown-item"
                  :class="{ active: rotationMode === key }"
                  @click="setRotationMode(key); rotationDropdownOpen = false">
                  {{ label }}
                </button>
              </li>
            </ul>
          </div>
        </template>

        <button
          v-if="showReset"
          type="button"
          class="overlay-toolbar-btn"
          title="Reset camera position"
          @click="handleReset()">
          <i class="bi bi-arrow-counterclockwise"></i>
        </button>
      </div>

      <button
        type="button"
        class="overlay-toolbar-btn"
        :class="{ 'overlay-toolbar-btn-active': costmapVisible }"
        @click="toggleCostMap()">
        Cost Map
      </button>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, onMounted, onBeforeUnmount } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import type { ControllerStateMessage, OccupancyGridMessage } from '@/types/websocket'
import type { OrientationMessage } from '@/types/coordinates'
import { quaternionToYaw } from '@/utils/map'
import { useRoverScene, CameraType, NUM_COSTMAP_BLOCKS } from '@/composables/useRoverScene'

const { onMessage, setupWebSocket, closeWebSocket } = useWebsocketStore()

const {
  setup: setupScene,
  dispose: disposeScene,
  setCamera,
  resetCamera,
  setNavAzimuth,
  updateCostMap,
  toggleCostMapVisibility,
  setCostMapVisibility,
  updateJoints,
  setRoverHeading,
} = useRoverScene()

enum ViewMode {
  Orbit = 'orbit',
  Follow = 'follow',
  Arm = 'arm',
  Top = 'top',
}

enum RotationMode {
  Manual = 'manual',
  North = 'north',
  FollowHeading = 'follow',
}

const viewLabels: Record<ViewMode, string> = {
  [ViewMode.Orbit]: 'Orbit',
  [ViewMode.Follow]: 'Follow',
  [ViewMode.Arm]: 'Arm',
  [ViewMode.Top]: 'Top-Down',
}

const rotationLabels: Record<RotationMode, string> = {
  [RotationMode.Manual]: 'Manual',
  [RotationMode.North]: 'True North',
  [RotationMode.FollowHeading]: 'Follow Heading',
}

const LS_VIEW_MODE = 'rover3d.viewMode'
const LS_ROTATION_MODE = 'rover3d.rotationMode'

const validViewModes = new Set(Object.values(ViewMode))
const validRotationModes = new Set(Object.values(RotationMode))

function loadEnum<T>(key: string, valid: Set<string>, fallback: T): T {
  const stored = localStorage.getItem(key)
  return stored && valid.has(stored) ? (stored as T) : fallback
}

const viewMode = ref<ViewMode>(loadEnum(LS_VIEW_MODE, validViewModes, ViewMode.Orbit))
const rotationMode = ref<RotationMode>(loadEnum(LS_ROTATION_MODE, validRotationModes, RotationMode.FollowHeading))
const viewDropdownOpen = ref(false)
const rotationDropdownOpen = ref(false)

const isTopMode = computed(() => viewMode.value === ViewMode.Top)
const showReset = computed(() => {
  if (viewMode.value === ViewMode.Orbit) return true
  if (isTopMode.value && rotationMode.value === RotationMode.Manual) return true
  return false
})

let manualAzimuth = 0
let roverMapPos = { x: 0, y: 0 }
let roverHeadingRad = 0
let dragStartX = 0
let dragStartAzimuth = 0
let isDragging = false

const viewToCameraType: Record<ViewMode, CameraType> = {
  [ViewMode.Orbit]: CameraType.Orbit,
  [ViewMode.Follow]: CameraType.Follow,
  [ViewMode.Arm]: CameraType.Arm,
  [ViewMode.Top]: CameraType.Top,
}

function switchView(mode: ViewMode) {
  viewMode.value = mode
  localStorage.setItem(LS_VIEW_MODE, mode)

  setCamera(viewToCameraType[mode])

  if (mode === ViewMode.Top) {
    setCostMapVisibility(costmapVisible.value)
    if (rotationMode.value === RotationMode.Manual) {
      manualAzimuth = 0
      setNavAzimuth(0)
    } else {
      updateTopDownCamera()
    }
  }
}

function handleReset() {
  if (isTopMode.value && rotationMode.value === RotationMode.Manual) {
    manualAzimuth = 0
    setNavAzimuth(0)
  } else {
    resetCamera()
  }
}

function closeDropdowns() {
  viewDropdownOpen.value = false
  rotationDropdownOpen.value = false
}

function onPointerDown(e: PointerEvent) {
  if (!isTopMode.value || rotationMode.value !== RotationMode.Manual) return
  if (e.button !== 0) return
  isDragging = true
  dragStartX = e.clientX
  dragStartAzimuth = manualAzimuth
  ;(e.target as HTMLElement).setPointerCapture(e.pointerId)
}

function onPointerMove(e: PointerEvent) {
  if (!isDragging) return
  const deltaX = e.clientX - dragStartX
  const sensitivity = 0.005
  manualAzimuth = dragStartAzimuth + deltaX * sensitivity
  setNavAzimuth(manualAzimuth)
}

function onPointerUp() {
  isDragging = false
}

const jointNameMap: Record<string, string> = {
  joint_a: 'chassis_to_arm_a',
  joint_b: 'arm_a_to_arm_b',
  joint_c: 'arm_b_to_arm_c',
  joint_de_pitch: 'arm_c_to_arm_d',
  joint_de_roll: 'arm_d_to_arm_e',
  gripper: 'gripper_link',
}

const lastKnownPositions: Record<string, number> = {}

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
  switchView(viewMode.value)
  setupWebSocket('nav')
})

onBeforeUnmount(() => {
  disposeScene()
  closeWebSocket('nav')
})

onMessage<ControllerStateMessage>('arm', 'arm_state', (msg) => {
  const joints = msg.names.map((name: string, index: number) => {
    const urdfName = jointNameMap[name] || name
    const incoming: number | null = (msg.positions as (number | null)[])[index] ?? null

    if (incoming !== null) {
      const mapped = urdfName === 'chassis_to_arm_a' ? incoming * -100 + 40 : incoming
      lastKnownPositions[urdfName] = mapped
    }

    const position = lastKnownPositions[urdfName] ?? 0
    return { name: urdfName, position }
  })

  updateJoints(joints)
})



function setRotationMode(mode: RotationMode) {
  rotationMode.value = mode
  localStorage.setItem(LS_ROTATION_MODE, mode)

  if (mode === RotationMode.Manual) {
    manualAzimuth = 0
    setNavAzimuth(0)
  } else {
    updateTopDownCamera()
  }
}

function updateTopDownCamera() {
  if (!isTopMode.value) return
  if (rotationMode.value === RotationMode.North) {
    setNavAzimuth(0)
  } else if (rotationMode.value === RotationMode.FollowHeading) {
    setNavAzimuth(roverHeadingRad - Math.PI / 2)
  }
}

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
  updateTopDownCamera()
})

onMessage<OrientationMessage>('nav', 'orientation', (msg) => {
  if (msg.position) {
    roverMapPos = { x: msg.position.x, y: msg.position.y }
  }
  roverHeadingRad = -quaternionToYaw(msg.orientation)
  setRoverHeading(roverHeadingRad)
  updateTopDownCamera()
})
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
</style>

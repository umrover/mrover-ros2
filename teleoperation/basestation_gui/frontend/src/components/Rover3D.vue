<template>
  <div class="flex flex-col w-full h-full">
    <div class="flex flex-wrap gap-1 mb-2 shrink-0">
      <button
        type="button"
        class="cmd-btn cmd-btn-sm cmd-btn-outline-control grow"
        :class="{ 'active': camera_type === 'default' }"
        @click="setCamera('default')">
        Default
      </button>
      <button
        type="button"
        class="cmd-btn cmd-btn-sm cmd-btn-outline-control grow"
        :class="{ 'active': camera_type === 'follow' }"
        @click="setCamera('follow')">
        Follow
      </button>
      <button
        type="button"
        class="cmd-btn cmd-btn-sm cmd-btn-outline-control grow"
        :class="{ 'active': camera_type === 'arm' }"
        @click="setCamera('arm')">
        Arm
      </button>
      <button
        type="button"
        class="cmd-btn cmd-btn-sm cmd-btn-outline-control grow"
        :class="{ 'active': camera_type === 'full arm' }"
        @click="setCamera('full arm')">
        Full Arm
      </button>
      <button
        type="button"
        class="cmd-btn cmd-btn-sm cmd-btn-outline-control grow"
        :class="{ 'active': camera_type === 'side arm' }"
        @click="setCamera('side arm')">
        Side Arm
      </button>
      <button
        type="button"
        class="cmd-btn cmd-btn-sm cmd-btn-outline-control grow"
        :class="{ 'active': camera_type === 'top' }"
        @click="setCamera('top')">
        Top Down
      </button>

      <button
        type="button"
        class="cmd-btn cmd-btn-sm cmd-btn-outline-secondary grow"
        @click="toggleCostMap()">
        Toggle Cost Map
      </button>

      <button
        type="button"
        class="cmd-btn cmd-btn-sm cmd-btn-outline-secondary grow"
        :class="{ 'active': doCostmapRotation }"
        @click="toggleCostmapRotation()">
        Relative Rotation
      </button>
    </div>
    <canvas class="webgl grow min-h-0 w-full"></canvas>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch, onMounted, onBeforeUnmount } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import type { ControllerStateMessage, IkFeedbackMessage, OccupancyGridMessage } from '@/types/websocket'
import type { NavMessage, OrientationMessage } from '@/types/coordinates'
import { quaternionToMapAngle } from '@/utils/map'
import { useRoverScene, NUM_COSTMAP_BLOCKS } from '@/composables/useRoverScene'

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

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
  websocketStore.setupWebSocket('nav')
})

onBeforeUnmount(() => {
  disposeScene()
  websocketStore.closeWebSocket('nav')
})

const armMessage = computed(() => messages.value['arm'])
const driveMessage = computed(() => messages.value['drive'])
const navMessage = computed(() => messages.value['nav'])

watch(armMessage, (msg: unknown) => {
  if (!msg || typeof msg !== 'object') return

  if ('type' in msg && msg.type === 'arm_state') {
    const typedMsg = msg as ControllerStateMessage
    const joints = typedMsg.names.map((name: string, index: number) => {
      const urdfName = jointNameMap[name] || name
      const rawPosition: number = typedMsg.positions[index] ?? 0
      const position = urdfName === 'chassis_to_arm_a'
        ? rawPosition * -100 + 40
        : rawPosition

      return { name: urdfName, position }
    })

    updateJoints(joints)
  } else if ('type' in msg && msg.type === 'ik_feedback') {
    const typedMsg = msg as IkFeedbackMessage
    updateIKTarget({
      x: typedMsg.pos.x * 100,
      y: typedMsg.pos.z * 100,
      z: typedMsg.pos.y * -100 + 20,
    })
  }
})

let roverMapPos = { x: 0, y: 0 }
const rover_bearing_deg = ref(0)
const doCostmapRotation = ref(true)

watch(driveMessage, (msg: unknown) => {
  if (!msg || typeof msg !== 'object' || !('data' in msg)) return

  const typedMsg = msg as OccupancyGridMessage
  const { resolution, width, height, origin } = typedMsg.info

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
        processedData[k] = typedMsg.data[row * width + col] ?? -1
      }
    }
  }

  updateCostMap(processedData)

  if (doCostmapRotation.value) {
    setCostMapRotation(Math.PI * rover_bearing_deg.value / 180 - Math.PI / 2)
  }
})

watch(navMessage, (msg) => {
  if (!msg) return
  const navMsg = msg as NavMessage
  if (navMsg.type === 'orientation') {
    const orientationMsg = navMsg as OrientationMessage
    rover_bearing_deg.value = quaternionToMapAngle(navMsg.orientation)
    if (orientationMsg.position) {
      roverMapPos = { x: orientationMsg.position.x, y: orientationMsg.position.y }
    }
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

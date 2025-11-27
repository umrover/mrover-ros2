<template>
  <div class="d-flex align-items-start justify-content-center w-100">
    <div class="d-flex flex-column gap-2 w-100">
      <div class="d-flex justify-content-between align-items-center">
        <h4 class="m-0">Gimbal Controls</h4>
        <IndicatorDot :is-active="controllerConnected" class="me-2" />
      </div>

      <div class="d-flex align-items-center justify-content-center gap-1">
        <span class="fw-semibold text-end" style="min-width: 40px; font-size: 14px">Pitch</span>
        <div class="btn-group btn-group-sm">
          <button class="btn btn-outline-secondary control-btn" @click="adjustGimbal('pitch', -10)">-10</button>
          <button class="btn btn-outline-secondary control-btn" @click="adjustGimbal('pitch', -5)">-5</button>
          <button class="btn btn-outline-secondary control-btn" @click="adjustGimbal('pitch', -1)">-1</button>
        </div>
        <span class="font-monospace fw-semibold text-center bg-white border rounded px-2 py-1" style="min-width: 40px; font-size: 14px">{{ pitchDegrees }}°</span>
        <div class="btn-group btn-group-sm">
          <button class="btn btn-outline-secondary control-btn" @click="adjustGimbal('pitch', 1)">+1</button>
          <button class="btn btn-outline-secondary control-btn" @click="adjustGimbal('pitch', 5)">+5</button>
          <button class="btn btn-outline-secondary control-btn" @click="adjustGimbal('pitch', 10)">+10</button>
        </div>
      </div>

      <div class="d-flex align-items-center justify-content-center gap-1">
        <span class="fw-semibold text-end" style="min-width: 40px; font-size: 14px">Yaw</span>
        <div class="btn-group btn-group-sm">
          <button class="btn btn-outline-secondary control-btn" @click="adjustGimbal('yaw', -10)">-10</button>
          <button class="btn btn-outline-secondary control-btn" @click="adjustGimbal('yaw', -5)">-5</button>
          <button class="btn btn-outline-secondary control-btn" @click="adjustGimbal('yaw', -1)">-1</button>
        </div>
        <span class="font-monospace fw-semibold text-center bg-white border rounded px-2 py-1" style="min-width: 40px; font-size: 14px">{{ yawDegrees }}°</span>
        <div class="btn-group btn-group-sm">
          <button class="btn btn-outline-secondary control-btn" @click="adjustGimbal('yaw', 1)">+1</button>
          <button class="btn btn-outline-secondary control-btn" @click="adjustGimbal('yaw', 5)">+5</button>
          <button class="btn btn-outline-secondary control-btn" @click="adjustGimbal('yaw', 10)">+10</button>
        </div>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, onMounted, onBeforeUnmount } from 'vue'
import { chassisAPI } from '@/utils/chassisAPI'
import { useWebsocketStore } from '@/stores/websocket'
import type { ControllerStateMessage } from '@/types/websocket'
import IndicatorDot from './IndicatorDot.vue'

const websocketStore = useWebsocketStore()

const controllerConnected = ref(false)
let interval: number | undefined = undefined

const UPDATE_HZ = 20

const gimbalJointState = computed((): ControllerStateMessage | null => {
  const messages = websocketStore.messages['chassis']
  if (!messages || !Array.isArray(messages)) return null
  const msg = messages.find((msg: { type: string }) => msg.type === 'gimbal_state')
  return msg ? (msg as ControllerStateMessage) : null
})

const pitchRadians = computed((): number => {
  const state = gimbalJointState.value
  if (!state || !state.name || !state.position) return 0
  const pitchIndex = state.name.indexOf('pitch')
  return pitchIndex >= 0 ? state.position[pitchIndex] : 0
})

const yawRadians = computed((): number => {
  const state = gimbalJointState.value
  if (!state || !state.name || !state.position) return 0
  const yawIndex = state.name.indexOf('yaw')
  return yawIndex >= 0 ? state.position[yawIndex] : 0
})

const pitchDegrees = computed((): string =>
  ((pitchRadians.value * 180) / Math.PI).toFixed(0),
)
const yawDegrees = computed((): string =>
  ((yawRadians.value * 180) / Math.PI).toFixed(0),
)

const adjustGimbal = async (
  joint: 'pitch' | 'yaw',
  adjustment: number,
): Promise<void> => {
  try {
    await chassisAPI.adjustGimbal(joint, adjustment, false)
  } catch (error) {
    console.error('Failed to adjust gimbal:', error)
  }
}

onMounted(() => {
  interval = window.setInterval(() => {
    const gamepads = navigator.getGamepads()
    const gamepad = gamepads.find(gamepad => gamepad && gamepad.id.includes('Xbox'))
    controllerConnected.value = !!gamepad
  }, 1000 / UPDATE_HZ)
})

onBeforeUnmount(() => {
  window.clearInterval(interval)
})
</script>

<style scoped>
.control-btn {
  min-width: 38px;
  font-size: 12px;
  padding: 2px 6px;
}
</style>

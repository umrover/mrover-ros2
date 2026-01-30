<template>
  <div class="d-flex align-items-start justify-content-center">
    <div class="d-flex flex-column gap-2">
      <div class="d-flex justify-content-between align-items-center">
        <h4 class="m-0">Gimbal Controls</h4>
        <div class="d-flex align-items-center">
          <i
            v-if="!hasServoState"
            ref="tooltipRef"
            class="bi bi-info-circle me-2"
            data-bs-toggle="tooltip"
            data-bs-placement="top"
            title="Changes not allowed until gimbal position received"
          ></i>
          <IndicatorDot :is-active="hasServoState" class="me-2" />
        </div>
      </div>

      <div class="d-flex align-items-center justify-content-center gap-1">
        <span class="fw-semibold text-end" style="min-width: 40px; font-size: 14px">Pitch</span>
        <div class="btn-group btn-group-sm">
          <button class="btn btn-outline-control control-btn" @click="adjustGimbal('pitch', -10)" :disabled="!hasServoState">-10</button>
          <button class="btn btn-outline-control control-btn" @click="adjustGimbal('pitch', -5)" :disabled="!hasServoState">-5</button>
          <button class="btn btn-outline-control control-btn" @click="adjustGimbal('pitch', -1)" :disabled="!hasServoState">-1</button>
        </div>
        <span class="font-monospace fw-semibold text-center bg-theme-card border rounded px-2 py-1" style="min-width: 40px; font-size: 14px">{{ pitchDegrees }}°</span>
        <div class="btn-group btn-group-sm">
          <button class="btn btn-outline-control control-btn" @click="adjustGimbal('pitch', 1)" :disabled="!hasServoState">+1</button>
          <button class="btn btn-outline-control control-btn" @click="adjustGimbal('pitch', 5)" :disabled="!hasServoState">+5</button>
          <button class="btn btn-outline-control control-btn" @click="adjustGimbal('pitch', 10)" :disabled="!hasServoState">+10</button>
        </div>
      </div>

      <div class="d-flex align-items-center justify-content-center gap-1">
        <span class="fw-semibold text-end" style="min-width: 40px; font-size: 14px">Yaw</span>
        <div class="btn-group btn-group-sm">
          <button class="btn btn-outline-control control-btn" @click="adjustGimbal('yaw', -10)" :disabled="!hasServoState">-10</button>
          <button class="btn btn-outline-control control-btn" @click="adjustGimbal('yaw', -5)" :disabled="!hasServoState">-5</button>
          <button class="btn btn-outline-control control-btn" @click="adjustGimbal('yaw', -1)" :disabled="!hasServoState">-1</button>
        </div>
        <span class="font-monospace fw-semibold text-center bg-theme-card border rounded px-2 py-1" style="min-width: 40px; font-size: 14px">{{ yawDegrees }}°</span>
        <div class="btn-group btn-group-sm">
          <button class="btn btn-outline-control control-btn" @click="adjustGimbal('yaw', 1)" :disabled="!hasServoState">+1</button>
          <button class="btn btn-outline-control control-btn" @click="adjustGimbal('yaw', 5)" :disabled="!hasServoState">+5</button>
          <button class="btn btn-outline-control control-btn" @click="adjustGimbal('yaw', 10)" :disabled="!hasServoState">+10</button>
        </div>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { computed, ref, shallowRef, onMounted, onUnmounted } from 'vue'
import { Tooltip } from 'bootstrap'
import { chassisAPI } from '@/utils/chassisAPI'
import { useWebsocketStore } from '@/stores/websocket'
import type { ControllerStateMessage } from '@/types/websocket'
import IndicatorDot from './IndicatorDot.vue'

const websocketStore = useWebsocketStore()

const tooltipRef = ref<HTMLElement | null>(null)
const tooltip = shallowRef<Tooltip | null>(null)

onMounted(() => {
  if (tooltipRef.value) {
    tooltip.value = new Tooltip(tooltipRef.value)
  }
})

onUnmounted(() => {
  tooltip.value?.dispose()
})

const gimbalJointState = computed((): ControllerStateMessage | null => {
  const messages = websocketStore.messages['chassis']
  console.log(messages)
  if (!messages || !Array.isArray(messages)) return null
  const msg = messages.find((msg: { type: string }) => msg.type === 'gimbal_controller_state')
  return msg ? (msg as ControllerStateMessage) : null
})

const hasServoState = computed((): boolean => {
  return gimbalJointState.value !== null
})

const pitchRadians = computed((): number => {
  const state = gimbalJointState.value
  if (!state || !state.names || !state.positions) return 0
  const pitchIndex = state.names.indexOf('pitch')
  return pitchIndex >= 0 ? (state.positions[pitchIndex] ?? 0) : 0
})

const yawRadians = computed((): number => {
  const state = gimbalJointState.value
  if (!state || !state.names || !state.positions) return 0
  const yawIndex = state.names.indexOf('yaw')
  return yawIndex >= 0 ? (state.positions[yawIndex] ?? 0) : 0
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
    const state = gimbalJointState.value
    if (!state || !state.names || !state.positions) {
      console.error('No gimbal state available')
      return
    }

    const jointIndex = state.names.indexOf(joint)
    if (jointIndex < 0) {
      console.error(`Joint ${joint} not found in state`)
      return
    }

    const currentPosition = state.positions[jointIndex]
    if (currentPosition === undefined) {
      console.error(`Position for joint ${joint} is undefined`)
      return
    }

    const adjustmentRadians = (adjustment * Math.PI) / 180
    const newPosition = currentPosition + adjustmentRadians

    await chassisAPI.adjustGimbal(joint, newPosition, true)
  } catch (error) {
    console.error('Failed to adjust gimbal:', error)
  }
}
</script>

<style scoped>
.control-btn {
  min-width: 38px;
  font-size: 12px;
  padding: 2px 6px;
}

.control-btn:disabled {
  background-color: var(--disabled-bg);
  border-color: var(--disabled-border);
  color: var(--disabled-text);
  opacity: 1;
  cursor: not-allowed;
}
</style>

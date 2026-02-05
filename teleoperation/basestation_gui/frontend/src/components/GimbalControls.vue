<template>
  <div class="d-flex flex-column gap-1 w-100 h-100 overflow-hidden">
    <div class="d-flex justify-content-between align-items-center w-100">
      <h4 class="component-header">Gimbal Controls</h4>
      <div class="d-flex align-items-center">
        <i
          v-if="!hasServoState"
          ref="tooltipRef"
          class="bi bi-info-circle me-1 small"
          data-bs-toggle="tooltip"
          data-bs-placement="top"
          title="Changes not allowed until gimbal position received"
        ></i>
        <IndicatorDot :is-active="hasServoState" />
      </div>
    </div>

    <div class="axis-block">
      <div class="axis-header">
        <span class="axis-label">Pitch</span>
        <span class="value-display">{{ pitchDegrees }}&deg;</span>
        <span class="axis-label-spacer"></span>
      </div>
      <div class="btn-row">
        <button class="btn btn-outline-control btn-sm control-btn" @click="adjustGimbal('pitch', -10)" :disabled="!hasServoState">-10</button>
        <button class="btn btn-outline-control btn-sm control-btn" @click="adjustGimbal('pitch', -5)" :disabled="!hasServoState">-5</button>
        <button class="btn btn-outline-control btn-sm control-btn" @click="adjustGimbal('pitch', -1)" :disabled="!hasServoState">-1</button>
        <button class="btn btn-outline-control btn-sm control-btn" @click="adjustGimbal('pitch', 1)" :disabled="!hasServoState">+1</button>
        <button class="btn btn-outline-control btn-sm control-btn" @click="adjustGimbal('pitch', 5)" :disabled="!hasServoState">+5</button>
        <button class="btn btn-outline-control btn-sm control-btn" @click="adjustGimbal('pitch', 10)" :disabled="!hasServoState">+10</button>
      </div>
    </div>

    <div class="axis-block">
      <div class="axis-header">
        <span class="axis-label">Yaw</span>
        <span class="value-display">{{ yawDegrees }}&deg;</span>
        <span class="axis-label-spacer"></span>
      </div>
      <div class="btn-row">
        <button class="btn btn-outline-control btn-sm control-btn" @click="adjustGimbal('yaw', -10)" :disabled="!hasServoState">-10</button>
        <button class="btn btn-outline-control btn-sm control-btn" @click="adjustGimbal('yaw', -5)" :disabled="!hasServoState">-5</button>
        <button class="btn btn-outline-control btn-sm control-btn" @click="adjustGimbal('yaw', -1)" :disabled="!hasServoState">-1</button>
        <button class="btn btn-outline-control btn-sm control-btn" @click="adjustGimbal('yaw', 1)" :disabled="!hasServoState">+1</button>
        <button class="btn btn-outline-control btn-sm control-btn" @click="adjustGimbal('yaw', 5)" :disabled="!hasServoState">+5</button>
        <button class="btn btn-outline-control btn-sm control-btn" @click="adjustGimbal('yaw', 10)" :disabled="!hasServoState">+10</button>
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
  adjustmentDegrees: number,
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

    const adjustmentRadians = (adjustmentDegrees * Math.PI) / 180
    const targetPosition = currentPosition + adjustmentRadians

    await chassisAPI.adjustGimbal(joint, targetPosition, true)
  } catch (error) {
    console.error('Failed to set gimbal position:', error)
  }
}
</script>

<style scoped>
.axis-block {
  display: flex;
  flex-direction: column;
  gap: 0.25rem;
}

.axis-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
}

.axis-label,
.axis-label-spacer {
  font-size: 0.7rem;
  font-weight: 600;
  text-transform: uppercase;
  color: var(--text-muted);
  width: 32px;
}

.axis-label-spacer {
  visibility: hidden;
}

.value-display {
  font-size: 0.8rem;
  font-weight: 600;
  background-color: var(--card-bg);
  border: 2px solid var(--input-border);
  border-radius: var(--cmd-radius-sm);
  padding: 0 0.5rem;
}

.btn-row {
  display: flex;
  gap: 0.25rem;
}

.control-btn {
  flex: 1;
  padding: 0.35rem 0;
  font-size: 0.7rem;
  font-weight: 600;
}

.control-btn:disabled {
  background-color: var(--disabled-bg);
  border-color: var(--disabled-border);
  color: var(--disabled-text);
  opacity: 1;
  cursor: not-allowed;
}
</style>

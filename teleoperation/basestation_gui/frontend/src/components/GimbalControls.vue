<template>
  <div class="flex flex-col gap-1 w-full h-full overflow-hidden" data-testid="pw-gimbal-controls">
    <div class="flex justify-between items-center w-full">
      <h4 class="component-header">Gimbal</h4>
      <div class="flex items-center">
        <i
          v-if="!hasServoState"
          class="bi bi-info-circle mr-1 text-sm"
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
      <div class="btn-row" data-testid="pw-gimbal-pitch-btns">
        <button class="cmd-btn cmd-btn-outline-control cmd-btn-sm control-btn" @click="adjustGimbal('pitch', -10)" :disabled="!hasServoState">-10</button>
        <button class="cmd-btn cmd-btn-outline-control cmd-btn-sm control-btn" @click="adjustGimbal('pitch', -5)" :disabled="!hasServoState">-5</button>
        <button class="cmd-btn cmd-btn-outline-control cmd-btn-sm control-btn" @click="adjustGimbal('pitch', -1)" :disabled="!hasServoState">-1</button>
        <button class="cmd-btn cmd-btn-outline-control cmd-btn-sm control-btn" @click="adjustGimbal('pitch', 1)" :disabled="!hasServoState">+1</button>
        <button class="cmd-btn cmd-btn-outline-control cmd-btn-sm control-btn" @click="adjustGimbal('pitch', 5)" :disabled="!hasServoState">+5</button>
        <button class="cmd-btn cmd-btn-outline-control cmd-btn-sm control-btn" @click="adjustGimbal('pitch', 10)" :disabled="!hasServoState">+10</button>
      </div>
    </div>

    <div class="axis-block">
      <div class="axis-header">
        <span class="axis-label">Yaw</span>
        <span class="value-display">{{ yawDegrees }}&deg;</span>
        <span class="axis-label-spacer"></span>
      </div>
      <div class="btn-row" data-testid="pw-gimbal-yaw-btns">
        <button class="cmd-btn cmd-btn-outline-control cmd-btn-sm control-btn" @click="adjustGimbal('yaw', -10)" :disabled="!hasServoState">-10</button>
        <button class="cmd-btn cmd-btn-outline-control cmd-btn-sm control-btn" @click="adjustGimbal('yaw', -5)" :disabled="!hasServoState">-5</button>
        <button class="cmd-btn cmd-btn-outline-control cmd-btn-sm control-btn" @click="adjustGimbal('yaw', -1)" :disabled="!hasServoState">-1</button>
        <button class="cmd-btn cmd-btn-outline-control cmd-btn-sm control-btn" @click="adjustGimbal('yaw', 1)" :disabled="!hasServoState">+1</button>
        <button class="cmd-btn cmd-btn-outline-control cmd-btn-sm control-btn" @click="adjustGimbal('yaw', 5)" :disabled="!hasServoState">+5</button>
        <button class="cmd-btn cmd-btn-outline-control cmd-btn-sm control-btn" @click="adjustGimbal('yaw', 10)" :disabled="!hasServoState">+10</button>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { computed } from 'vue'
import { chassisAPI } from '@/utils/chassisAPI'
import { useWebsocketStore } from '@/stores/websocket'
import type { ControllerStateMessage } from '@/types/websocket'
import IndicatorDot from './IndicatorDot.vue'

const websocketStore = useWebsocketStore()

const gimbalJointState = computed((): ControllerStateMessage | null => {
  const msg = websocketStore.messages['chassis']
  if (!msg) return null
  const typedMsg = msg as ControllerStateMessage
  return typedMsg.type === 'gimbal_controller_state' ? typedMsg : null
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
  width: 32px;
  font-size: 0.7rem;
  font-weight: 600;
  color: var(--text-muted);
  text-transform: uppercase;
}

.axis-label-spacer {
  visibility: hidden;
}

.value-display {
  padding: 0 0.5rem;
  font-size: 0.8rem;
  font-weight: 600;
  background-color: var(--card-bg);
  border: 2px solid var(--input-border);
  border-radius: var(--cmd-radius-sm);
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
  color: var(--disabled-text);
  cursor: not-allowed;
  background-color: var(--disabled-bg);
  border-color: var(--disabled-border);
  opacity: 1;
}
</style>

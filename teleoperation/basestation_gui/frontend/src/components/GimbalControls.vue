<template>
  <div class="flex flex-col gap-1 w-full h-full overflow-hidden" data-testid="pw-gimbal-controls">
    <div class="flex justify-between items-center w-full">
      <h4 class="component-header">Gimbal</h4>
      <IndicatorDot :is-active="hasServoState && atTarget" />
    </div>

    <div v-for="joint in JOINTS" :key="joint" class="axis-block">
      <div class="axis-header">
        <span class="axis-label">{{ joint }}</span>
        <span class="value-display">{{ jointDegrees(joint) }}&deg;</span>
        <span class="axis-label-spacer"></span>
      </div>
      <div class="btn-row" :data-testid="`pw-gimbal-${joint}-btns`">
        <button
          v-for="delta in DELTAS"
          :key="delta"
          class="cmd-btn cmd-btn-outline-control cmd-btn-sm control-btn"
          :class="{ 'cmd-btn-secondary': pendingJoint === joint }"
          :disabled="!hasServoState || isLoading"
          @click="adjustGimbal(joint, delta)"
        >
          {{ delta > 0 ? `+${delta}` : delta }}
        </button>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed } from 'vue'
import { chassisAPI } from '@/utils/chassisAPI'
import { useWebsocketStore } from '@/stores/websocket'
import type { ControllerStateMessage } from '@/types/websocket'
import IndicatorDot from './IndicatorDot.vue'

type Joint = 'pitch' | 'yaw'

const JOINTS: Joint[] = ['pitch', 'yaw']
const DELTAS = [-10, -5, -1, 1, 5, 10]
const DEG_TO_RAD = Math.PI / 180
const RAD_TO_DEG = 180 / Math.PI

const websocketStore = useWebsocketStore()
const isLoading = ref(false)
const atTarget = ref(false)
const pendingJoint = ref<Joint | null>(null)

const gimbalJointState = computed((): ControllerStateMessage | null => {
  const msg = websocketStore.messages['chassis']
  if (!msg) return null
  const typedMsg = msg as ControllerStateMessage
  return typedMsg.type === 'gimbal_controller_state' ? typedMsg : null
})

const hasServoState = computed(() => gimbalJointState.value !== null)

function jointRadians(joint: Joint): number {
  const state = gimbalJointState.value
  if (!state?.names || !state.positions) return 0
  const idx = state.names.indexOf(joint)
  return idx >= 0 ? (state.positions[idx] ?? 0) : 0
}

function jointDegrees(joint: Joint): string {
  return (jointRadians(joint) * RAD_TO_DEG).toFixed(0)
}

async function adjustGimbal(joint: Joint, deltaDegrees: number) {
  if (isLoading.value || !hasServoState.value) return

  const currentRadians = jointRadians(joint)
  const targetRadians = currentRadians + deltaDegrees * DEG_TO_RAD

  pendingJoint.value = joint
  isLoading.value = true

  try {
    const result = await chassisAPI.adjustGimbal(joint, targetRadians, true)
    atTarget.value = !!result.at_tgt
  } catch {
    atTarget.value = false
  } finally {
    isLoading.value = false
    pendingJoint.value = null
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

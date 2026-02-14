<template>
  <div class="flex flex-col gap-1 w-full h-full overflow-hidden items-center" data-testid="pw-gimbal-controls">
    <div class="flex justify-between items-center w-full">
      <h4 class="component-header">Gimbal</h4>
      <IndicatorDot :is-active="hasServoState && atTarget" />
    </div>

    <div class="gimbal-ring-container">
      <!-- Outer ring: +/-10 -->
      <button
        v-for="btn in outerButtons"
        :key="`outer-${btn.joint}-${btn.delta}`"
        class="ring-btn outer"
        :style="ringPosition(btn.angle, outerRadius)"
        :disabled="btnDisabled"
        :class="btnClass(btn.joint)"
        :data-testid="`pw-gimbal-${btn.joint}-btns`"
        @click="adjustGimbal(btn.joint, btn.delta)"
      >
        {{ btn.label }}
      </button>

      <!-- Inner ring: +/-1 -->
      <button
        v-for="btn in innerButtons"
        :key="`inner-${btn.joint}-${btn.delta}`"
        class="ring-btn inner"
        :style="ringPosition(btn.angle, innerRadius)"
        :disabled="btnDisabled"
        :class="btnClass(btn.joint)"
        @click="adjustGimbal(btn.joint, btn.delta)"
      >
        {{ btn.label }}
      </button>

      <!-- Center display -->
      <div class="ring-center">
        <div class="text-[10px] text-muted uppercase">Pitch</div>
        <div class="text-sm font-semibold">{{ jointDegrees('pitch') }}°</div>
        <div class="text-[10px] text-muted uppercase mt-0.5">Yaw</div>
        <div class="text-sm font-semibold">{{ jointDegrees('yaw') }}°</div>
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

interface RingButton {
  joint: Joint
  delta: number
  label: string
  angle: number
}

const DEG_TO_RAD = Math.PI / 180
const RAD_TO_DEG = 180 / Math.PI

const SIZE = 180
const outerRadius = 76
const innerRadius = 46
const outerBtnSize = 32
const innerBtnSize = 26

const outerButtons: RingButton[] = [
  { joint: 'pitch', delta: -10, label: '-10', angle: 270 },
  { joint: 'yaw', delta: 10, label: '+10', angle: 0 },
  { joint: 'pitch', delta: 10, label: '+10', angle: 90 },
  { joint: 'yaw', delta: -10, label: '-10', angle: 180 },
]

const innerButtons: RingButton[] = [
  { joint: 'pitch', delta: -1, label: '-1', angle: 270 },
  { joint: 'yaw', delta: 1, label: '+1', angle: 0 },
  { joint: 'pitch', delta: 1, label: '+1', angle: 90 },
  { joint: 'yaw', delta: -1, label: '-1', angle: 180 },
]

function ringPosition(angleDeg: number, radius: number) {
  const rad = angleDeg * DEG_TO_RAD
  const cx = SIZE / 2
  const cy = SIZE / 2
  const x = cx + radius * Math.cos(rad)
  const y = cy + radius * Math.sin(rad)
  return { left: `${x}px`, top: `${y}px` }
}

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
const btnDisabled = computed(() => !hasServoState.value || isLoading.value)

function btnClass(joint: Joint) {
  return { 'cmd-btn-secondary': pendingJoint.value === joint }
}

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

  const targetRadians = jointRadians(joint) + deltaDegrees * DEG_TO_RAD

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
.gimbal-ring-container {
  position: relative;
  width: v-bind(SIZE + 'px');
  height: v-bind(SIZE + 'px');
  flex-shrink: 0;
}

.ring-btn {
  position: absolute;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: 600;
  color: var(--control-primary);
  cursor: pointer;
  background: transparent;
  border: var(--cmd-border-width) solid var(--control-primary);
  border-radius: 50%;
  transition: all var(--cmd-transition);
  transform: translate(-50%, -50%);
}

.ring-btn:hover:not(:disabled) {
  color: #fff;
  background-color: var(--control-primary);
}

.ring-btn:disabled {
  color: var(--disabled-text);
  cursor: not-allowed;
  background-color: var(--disabled-bg);
  border-color: var(--disabled-border);
}

.ring-btn.outer {
  width: v-bind(outerBtnSize + 'px');
  height: v-bind(outerBtnSize + 'px');
  font-size: 0.65rem;
}

.ring-btn.inner {
  width: v-bind(innerBtnSize + 'px');
  height: v-bind(innerBtnSize + 'px');
  font-size: 0.6rem;
}

.ring-center {
  position: absolute;
  top: 50%;
  left: 50%;
  transform: translate(-50%, -50%);
  display: flex;
  flex-direction: column;
  align-items: center;
  line-height: 1.2;
}
</style>

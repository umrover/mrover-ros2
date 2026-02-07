<template>
  <div class="d-flex flex-column gap-2 h-100">
    <div class="d-flex justify-content-between align-items-center">
      <h4 class="component-header m-0">Arm Controls</h4>
      <IndicatorDot :is-active="connected" class="me-2" />
    </div>
    <div class="btn-group w-100 border-2" role="group" aria-label="Arm mode selection" data-testid="pw-arm-mode-buttons">
        <button
          type="button"
          class="btn btn-sm border-2"
          :class="mode === 'disabled' ? 'btn-danger' : 'btn-outline-danger'"
          data-testid="pw-arm-mode-disabled"
          @click="newRAMode('disabled')"
        >
          Disabled
        </button>
        <button
          type="button"
          class="btn btn-sm border-2"
          :class="mode === 'throttle' ? 'btn-success' : 'btn-outline-success'"
          data-testid="pw-arm-mode-throttle"
          @click="newRAMode('throttle')"
        >
          Throttle
        </button>
        <button
          type="button"
          class="btn btn-sm border-2"
          :class="mode === 'ik-pos' ? 'btn-success' : 'btn-outline-success'"
          data-testid="pw-arm-mode-ik-pos"
          @click="newRAMode('ik-pos')"
        >
          IK Pos
        </button>
        <button
          type="button"
          class="btn btn-sm border-2"
          :class="mode === 'ik-vel' ? 'btn-success' : 'btn-outline-success'"
          data-testid="pw-arm-mode-ik-vel"
          @click="newRAMode('ik-vel')"
        >
          IK Vel
        </button>
      </div>

    <!-- TODO(stow): Add stow button. Gray out while stowing, show distance to target.
         On convergence or timeout, call setRAMode("disabled"). -->
    <!-- <button class="btn btn-sm btn-outline-warning border-2 w-100" :disabled="isStowing" @click="stowArm">
      Stow
    </button> -->

    <GamepadDisplay :axes="axes" :buttons="buttons" layout="horizontal" class="flex-grow-1 min-height-0" />
  </div>
</template>

<script lang="ts" setup>
import { ref, onMounted, onBeforeUnmount } from 'vue'
import { armAPI } from '@/utils/api'
import { useGamepadPolling } from '@/composables/useGamepadPolling'
import GamepadDisplay from './GamepadDisplay.vue'
import IndicatorDot from './IndicatorDot.vue'

const mode = ref('disabled')

const { connected, axes, buttons } = useGamepadPolling({
  controllerIdFilter: 'Microsoft',
  topic: 'arm',
  messageType: 'ra_controller',
})

const keyDown = async (event: { key: string }) => {
  if (event.key === ' ') {
    await newRAMode('disabled')
  }
}

onMounted(() => {
  document.addEventListener('keydown', keyDown)
})

onBeforeUnmount(() => {
  document.removeEventListener('keydown', keyDown)
})

// TODO(stow): Implement stow arm. /arm_ik reports actual EE position in 3-space.
//
// Approach:
//   1. Call armAPI.stowArm() -- REST sets ra_mode to "stow", returns target {x,y,z}
//   2. The 15 Hz gamepad polling continues sending ra_controller messages via WebSocket.
//      In "stow" mode, send_ra_controls() publishes STOW_POSITION instead of gamepad input,
//      keeping the arm watchdog timer satisfied.
//   3. Watch websocketStore.messages['arm'] for 'ik_target' messages (streamed from /arm_ik
//      ROS topic at 30 Hz). Compute Euclidean distance from current EE position to stow target.
//   4. On convergence (distance < STOW_THRESHOLD): call setRAMode("disabled"), show success.
//      On timeout (STOW_TIMEOUT_MS): call setRAMode("disabled"), show failure.
//
// const STOW_THRESHOLD = 0.01
// const STOW_TIMEOUT_MS = 30_000
//
// const isStowing = ref(false)
// const stowTarget = ref<{ x: number; y: number; z: number } | null>(null)
// const stowDistance = ref<number | null>(null)
// let stowTimeoutId: number | undefined
//
// const euclideanDistance = (a: { x: number; y: number; z: number }, b: { x: number; y: number; z: number }) =>
//   Math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
//
// const stowArm = async () => {
//   // TODO(stow)
// }
//
// const completeStow = async (reason: 'success' | 'timeout') => {
//   // TODO(stow)
// }

const newRAMode = async (newMode: string) => {
  try {
    mode.value = newMode
    const data = await armAPI.setRAMode(mode.value)
    if (data.status === 'success' && data.mode) {
      mode.value = data.mode
    }
  } catch (error) {
    console.error('Failed to set arm mode:', error)
  }
}
</script>

<style scoped>
.min-height-0 {
  min-height: 0;
}
</style>

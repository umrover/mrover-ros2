<template>
  <div class="flex flex-col gap-2 h-full">
    <div class="flex justify-between items-center">
      <h4 class="component-header">Arm Controls</h4>
      <p 
      class="text-danger"
      :class="forcing_limit === true ? 'visible' : 'invisible'">
        Limit Reached!
      </p>
      <IndicatorDot :is-active="connected" class="mr-2" />
    </div>
    <div class="flex w-full" role="group" aria-label="Arm mode selection" data-testid="pw-arm-mode-buttons">
      <div class="btn-group-connected w-full">
        <button
          type="button"
          class="btn btn-sm flex-1"
          :class="mode === 'disabled' ? 'btn-danger' : 'btn-outline-danger'"
          data-testid="pw-arm-mode-disabled"
          @click="newRAMode('disabled')"
        >
          Disabled
        </button>
        <button
          type="button"
          class="btn btn-sm flex-1"
          :class="mode === 'throttle' ? 'btn-success' : 'btn-outline-success'"
          data-testid="pw-arm-mode-throttle"
          @click="newRAMode('throttle')"
        >
          Throttle
        </button>
        <button
          type="button"
          class="btn btn-sm flex-1"
          :class="mode === 'ik-pos' ? 'btn-success' : 'btn-outline-success'"
          data-testid="pw-arm-mode-ik-pos"
          @click="newRAMode('ik-pos')"
        >
          IK Pos
        </button>
        <button
          type="button"
          class="btn btn-sm flex-1"
          :class="mode === 'ik-vel' ? 'btn-success' : 'btn-outline-success'"
          data-testid="pw-arm-mode-ik-vel"
          @click="newRAMode('ik-vel')"
        >
          IK Vel
        </button>
      </div>
    </div>
    <GamepadDisplay
      :axes="axes"
      :buttons="buttons"
      layout="horizontal"
      class="grow min-h-0"
    />
  </div>
</template>

<script lang="ts" setup>
import { ref, onMounted, onBeforeUnmount } from 'vue'
import { armAPI } from '@/utils/api'
import { useGamepadPolling } from '@/composables/useGamepadPolling'
import { useWebsocketStore } from '@/stores/websocket'
import type { ControllerStateMessage } from '@/types/websocket'
import GamepadDisplay from './GamepadDisplay.vue'
import IndicatorDot from './IndicatorDot.vue'

const { onMessage } = useWebsocketStore()

const mode = ref('disabled')
const forcing_limit = ref(false)

const { connected, axes, buttons, vibrationActuator } = useGamepadPolling({
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

const check_horizontal_axis_limit = (
  idx: number,
  limit_status: number,
  dead_radius: number = 0.05,
): boolean => {
  if (axes.value[idx] > dead_radius && limit_status == 2) return true
  if (axes.value[idx] < -dead_radius && limit_status == 1) return true
  return false
}

const check_vertical_axis_limit = (
  idx: number,
  limit_status: number,
  dead_radius: number = 0.05,
): boolean => {
  if (axes.value[idx] < -dead_radius && limit_status == 1) return true
  if (axes.value[idx] > dead_radius && limit_status == 2) return true
  return false
}

const check_button_limit = (
  left_idx: number,
  right_idx: number,
  limit_status: number,
): boolean => {
  if (buttons.value[left_idx] && limit_status == 1) return true
  if (buttons.value[right_idx] && limit_status == 2) return true
  return false
}

const VIBRATION_THRESHOLD = 0.8

onMessage<ControllerStateMessage>('arm', 'arm_state', msg => {
  const left_horiz_limit = check_horizontal_axis_limit(0, msg.limits_hit[0])
  const left_vert_limit = check_vertical_axis_limit(1, msg.limits_hit[1])
  const right_vert_limit = check_vertical_axis_limit(3, msg.limits_hit[2])
  const bumper_limit = check_button_limit(4, 5, msg.limits_hit[4])

  const intentional_limit =
    (left_horiz_limit && Math.abs(axes.value[0]) > VIBRATION_THRESHOLD) ||
    (left_vert_limit && Math.abs(axes.value[1]) > VIBRATION_THRESHOLD) ||
    (right_vert_limit && Math.abs(axes.value[3]) > VIBRATION_THRESHOLD) ||
    bumper_limit

  forcing_limit.value = intentional_limit

  if (intentional_limit && vibrationActuator.value) {
    vibrationActuator.value.playEffect('dual-rumble', {
      startDelay: 0,
      duration: 100,
      weakMagnitude: 0.1,
      strongMagnitude: 0,
    })
  }
})
</script>

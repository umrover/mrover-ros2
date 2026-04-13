<template>
  <div class="flex flex-col gap-2 h-full">
    <div class="flex justify-between items-center">
      <h4 class="component-header">Arm Controls</h4>
      <p class="text-danger" :class="forcing_limit ? 'visible' : 'invisible'">Limit Reached!</p>
      <IndicatorDot :is-active="connected" class="mr-2" />
    </div>
    <div class="btn-group w-full" role="group" aria-label="Arm mode selection" data-testid="pw-arm-mode-buttons">
        <button
          type="button"
          class="cmd-btn cmd-btn-sm"
          :class="mode === 'disabled' ? 'cmd-btn-danger' : 'cmd-btn-outline-danger'"
          data-testid="pw-arm-mode-disabled"
          @click="newRAMode('disabled')"
        >
          Disabled
        </button>
        <button
          type="button"
          class="cmd-btn cmd-btn-sm"
          :class="mode === 'throttle' ? 'cmd-btn-success' : 'cmd-btn-outline-success'"
          data-testid="pw-arm-mode-throttle"
          @click="newRAMode('throttle')"
        >
          Throttle
        </button>
        <button
          type="button"
          class="cmd-btn cmd-btn-sm"
          :class="mode === 'ik-pos' ? 'cmd-btn-success' : 'cmd-btn-outline-success'"
          data-testid="pw-arm-mode-ik-pos"
          @click="newRAMode('ik-pos')"
        >
          IK Pos
        </button>
        <button
          type="button"
          class="cmd-btn cmd-btn-sm"
          :class="mode === 'ik-vel' ? 'cmd-btn-success' : 'cmd-btn-outline-success'"
          data-testid="pw-arm-mode-ik-vel"
          @click="newRAMode('ik-vel')"
        >
          IK Vel
        </button>
      </div>
    <GamepadDisplay :axes="axes" :buttons="buttons" layout="horizontal" class="grow min-h-0" />
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

const check_horizontal_axis_limit = (idx: number, limit_status: number, dead_radius: number = 0.05) => {
  if (axes.value[idx] > dead_radius && limit_status == 1) {
    forcing_limit.value = true
  } else if (axes.value[idx] < -dead_radius && limit_status == 2) {
    forcing_limit.value = true
  }
}

const check_vertical_axis_limit = (idx: number, limit_status: number, dead_radius: number = 0.05) => {
  if (axes.value[idx] < -dead_radius && limit_status == 1) {
    forcing_limit.value = true
  } else if (axes.value[idx] > dead_radius && limit_status == 2) {
    forcing_limit.value = true
  }
}

const check_button_limit = (left_idx: number, right_idx: number, limit_status: number) => {
  if (buttons.value[left_idx] && limit_status == 1) {
    forcing_limit.value = true
  } else if (buttons.value[right_idx] && limit_status == 2) {
    forcing_limit.value = true
  }
}

onMessage<ControllerStateMessage>('arm', 'arm_state', (msg) => {
  forcing_limit.value = false

  // Left joystick horizontal (joint_a)
  check_horizontal_axis_limit(0, msg.limits_hit[0])

  // Left joystick vertical (joint_b)
  check_vertical_axis_limit(1, msg.limits_hit[1])

  // Right joystick vertical (joint_c)
  check_vertical_axis_limit(3, msg.limits_hit[2])

  // Bumpers (joint_de_pitch)
  check_button_limit(4, 5, msg.limits_hit[4])

  if (forcing_limit.value && vibrationActuator.value) {
    vibrationActuator.value.playEffect('dual-rumble', {
      startDelay: 0,
      duration: 100,
      weakMagnitude: 0.1,
      strongMagnitude: 0,
    })
  }
})
</script>

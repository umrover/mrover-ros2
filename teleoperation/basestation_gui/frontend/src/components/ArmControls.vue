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

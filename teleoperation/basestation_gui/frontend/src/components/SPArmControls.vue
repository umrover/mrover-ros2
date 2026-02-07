<template>
  <div class="d-flex flex-column gap-2 h-100">
    <div class="d-flex justify-content-between align-items-center">
      <h4 class="component-header">SP Arm Controls</h4>
      <IndicatorDot :is-active="connected" />
    </div>
    <GamepadDisplay :axes="axes" :buttons="buttons" class="flex-grow-1 min-height-0" />
  </div>
</template>

<script lang="ts" setup>
import { useGamepadPolling } from '@/composables/useGamepadPolling'
import GamepadDisplay from './GamepadDisplay.vue'
import IndicatorDot from './IndicatorDot.vue'

const { connected, axes, buttons } = useGamepadPolling({
  controllerIdFilter: 'Microsoft',
  topic: 'science',
  messageType: 'sp_controller',
})
</script>

<style scoped>
.min-height-0 {
  min-height: 0;
}
</style>

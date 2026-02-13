<template>
  <div class="flex flex-col gap-2 h-full">
    <div class="flex justify-between items-center">
      <h4 class="component-header">SP Arm Controls</h4>
      <IndicatorDot :is-active="connected" />
    </div>
    <GamepadDisplay :axes="axes" :buttons="buttons" class="grow min-h-0" />
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


<template>
  <div class="flex flex-col w-full h-full">
    <div class="flex justify-between items-center w-full">
      <h4 class="component-header">Drive Controls</h4>
      <IndicatorDot :is-active="connected" />
    </div>
  </div>
</template>

<script lang='ts' setup>
import { useGamepadPolling } from '@/composables/useGamepadPolling'
import IndicatorDot from './IndicatorDot.vue'

const { connected } = useGamepadPolling({
  controllerIdFilter: 'Thrustmaster',
  topic: 'drive',
  messageType: 'joystick',
  transformAxes: (axes) => axes.map((v, i) => i === 1 ? -v : v),
})
</script>

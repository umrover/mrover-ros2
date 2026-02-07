<template>
  <div class="d-flex flex-column w-100 h-100">
    <div class="d-flex justify-content-between align-items-center w-100">
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

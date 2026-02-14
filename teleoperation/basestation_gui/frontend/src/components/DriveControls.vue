<template>
  <div class="flex flex-col w-full h-full">
    <div class="flex justify-between items-center w-full">
      <h4 class="component-header">Drive</h4>
      <IndicatorDot :is-active="connected" />
    </div>
  </div>
</template>

<script lang='ts' setup>
import { useGamepad } from '@/composables/useGamepad'
import IndicatorDot from './IndicatorDot.vue'

const { connected } = useGamepad({
  controllerIdFilter: 'Thrustmaster',
  transformAxes: (axes) => axes.map((v, i) => i === 1 ? -v : v),
  ws: { topic: 'drive', messageType: 'joystick' },
})
</script>

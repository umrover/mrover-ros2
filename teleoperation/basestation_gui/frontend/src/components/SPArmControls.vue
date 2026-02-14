<template>
  <div class="flex flex-col gap-2 h-full">
    <div class="flex justify-between items-center">
      <h4 class="component-header">SP Arm</h4>
      <i class="bi bi-dpad-fill" :class="connected ? 'text-cmd-success' : 'text-muted'" />
    </div>
    <GamepadDisplay :axes="axes" :buttons="buttons" class="grow min-h-0" />
  </div>
</template>

<script lang="ts" setup>
import { useGamepad } from '@/composables/useGamepad'
import GamepadDisplay from './GamepadDisplay.vue'

const { connected, axes, buttons } = useGamepad({
  controllerIdFilter: 'Microsoft',
  ws: { topic: 'science', messageType: 'sp_controller' },
})
</script>


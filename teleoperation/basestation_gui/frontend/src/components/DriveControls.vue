<template>
  <div class="d-flex flex-column align-items-start h-100">
    <div class="d-flex align-items-center gap-2">
      <h4 class="m-0">Drive Controls</h4>
      <IndicatorDot :is-active="controllerConnected" />
    </div>
  </div>
</template>

<script lang='ts' setup>
import { ref, onMounted, onBeforeUnmount } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import IndicatorDot from './IndicatorDot.vue'

const websocketStore = useWebsocketStore()

const controllerConnected = ref(false)
let interval: number | undefined = undefined

const UPDATE_HZ = 20

onMounted(() => {
  interval = window.setInterval(() => {
    const gamepads = navigator.getGamepads()
    const gamepad = gamepads.find(gamepad => gamepad && gamepad.id.includes('Thrustmaster'))
    controllerConnected.value = !!gamepad
    if (!gamepad) return

    const inverse_axes = gamepad.axes.map((value, index) => index === 1 ? -value : value)

    websocketStore.sendMessage('drive', {
      type: 'joystick',
      axes: inverse_axes,
      buttons: gamepad.buttons.map(button => button.value)
    })
  }, 1000 / UPDATE_HZ)
})

onBeforeUnmount(() => {
  window.clearInterval(interval)
})
</script>

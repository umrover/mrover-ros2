<template>
  <div class="d-flex flex-fill justify-content-between align-items-center gap-2 h-100 p-2">
    <h4 class="m-0 font-monospace align-content-center">Drive</h4>
    <span
      class="rounded-circle"
      :class="controllerConnected ? 'bg-success' : 'bg-danger'"
      style="width: 20px; height: 20px;"
    ></span>
  </div>
</template>

<script lang='ts' setup>
import { ref, onMounted, onBeforeUnmount } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'

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

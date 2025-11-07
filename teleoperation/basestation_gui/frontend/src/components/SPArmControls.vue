<template>
  <div class="wrap flex-column justify-content-between">
    <div class="d-flex justify-content-between align-items-center w-100">
      <h3 class="m-0 p-0">SA Arm Controls</h3>
      <div
        class="rounded-circle me-2"
        :class="controllerConnected ? 'bg-success' : 'bg-danger'"
        style="width: 16px; height: 16px"
      ></div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, onMounted, onBeforeUnmount } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'

const UPDATE_HZ = 20

const websocketStore = useWebsocketStore()

const gamepadConnected = ref(false)
const controllerConnected = computed(() => gamepadConnected.value)

let interval: number | undefined

onMounted(() => {
  interval = window.setInterval(() => {
    const gamepads = navigator.getGamepads()
    const gamepad = gamepads.find(
      gamepad => gamepad && gamepad.id.includes('Microsoft')
    )
    gamepadConnected.value = !!gamepad
    if (!gamepad) return

    websocketStore.sendMessage('arm', {
      type: 'sa_controller',
      axes: gamepad.axes,
      buttons: gamepad.buttons.map(button => button.value),
    })
  }, 1000 / UPDATE_HZ)
})

onBeforeUnmount(() => {
  if (interval !== undefined) {
    window.clearInterval(interval)
  }
})
</script>

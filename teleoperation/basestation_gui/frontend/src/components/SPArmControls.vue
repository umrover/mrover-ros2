<template>
  <div class="sp-controls d-flex flex-column align-items-center gap-2">
    <div class="d-flex justify-content-between align-items-center w-100">
      <h4 class="m-0">SP Arm Controls</h4>
      <IndicatorDot :is-active="controllerConnected" class="me-2" />
    </div>
    <GamepadDisplay :axes="axes" :buttons="buttons" />
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, onMounted, onBeforeUnmount } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import GamepadDisplay from './GamepadDisplay.vue'
import IndicatorDot from './IndicatorDot.vue'

const UPDATE_HZ = 20

const websocketStore = useWebsocketStore()

const gamepadConnected = ref(false)
const controllerConnected = computed(() => gamepadConnected.value)

const axes = ref<number[]>([0, 0, 0, 0])
const buttons = ref<number[]>(new Array(17).fill(0))

let interval: number | undefined

onMounted(() => {
  interval = window.setInterval(() => {
    const gamepads = navigator.getGamepads()
    const gamepad = gamepads.find(
      gamepad => gamepad && gamepad.id.includes('Microsoft'),
    )
    gamepadConnected.value = !!gamepad
    if (!gamepad) return

    axes.value = [...gamepad.axes]
    buttons.value = gamepad.buttons.map(button => button.value)

    websocketStore.sendMessage('science', {
      type: 'sp_controller',
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

<style scoped>
.sp-controls {
  width: 100%;
}
</style>

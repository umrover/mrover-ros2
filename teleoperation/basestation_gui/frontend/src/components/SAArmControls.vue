<template>
  <div class="wrap p-2 flex-column justify-content-between">
    <h3 class="m-0 p-0">Arm Controls</h3>
    <div class="btn-group m-0 p-0" role="group" aria-label="Arm Modes">
      <button
        type="button"
        class="btn"
        :class="mode === 'disabled' ? 'btn-danger' : 'btn-outline-danger'"
        @click="mode = 'disabled'"
      >
        Disabled
      </button>
      <button
        type="button"
        class="btn"
        :class="mode === 'throttle' ? 'btn-success' : 'btn-outline-success'"
        @click="mode = 'throttle'"
      >
        Throttle
      </button>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, onMounted, onBeforeUnmount, defineProps } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'

const props = defineProps({
  currentSite: {
    type: Number,
    required: true,
  },
})

const websocketStore = useWebsocketStore()

const mode = ref('disabled')
const corer_position = ref(0)
const plunger_position = ref(0)
const sensor_height = ref(5.36)
const plunger_height = ref(5.5)

let interval: number | undefined = undefined

const UPDATE_HZ = 20

onMounted(() => {
  interval = window.setInterval(() => {
    const gamepads = navigator.getGamepads()
    const gamepad = gamepads.find(
      gamepad => gamepad && gamepad.id.includes('Microsoft')
    )
    if (!gamepad) return

    websocketStore.sendMessage('arm', {
      type: 'sa_controller',
      axes: gamepad.axes,
      buttons: gamepad.buttons.map(button => button.value),
    })
    websocketStore.sendMessage('arm', {
      type: 'sa_mode',
      mode: mode.value,
    })
  }, 1000 / UPDATE_HZ)
})

onBeforeUnmount(() => {
  window.clearInterval(interval)
})
</script>

<style scoped>
.wrap {
  display: inline-flex;
  flex-direction: column;
  width: auto;
}
</style>

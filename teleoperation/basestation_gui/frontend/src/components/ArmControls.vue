<template>
  <div class="d-flex flex-column align-items-center w-100">
    <div class="d-flex flex-column gap-2">
      <div class="d-flex justify-content-between align-items-center">
        <h4 class="m-0">Arm Controls</h4>
        <div
          class="rounded-circle me-2"
          :class="controllerConnected ? 'bg-success' : 'bg-danger'"
          style="width: 16px; height: 16px"
        ></div>
      </div>
      <div
        class="btn-group d-flex justify-content-between"
        role="group"
        aria-label="Arm mode selection"
      >
        <button
          type="button"
          class="btn flex-fill"
          :class="mode === 'disabled' ? 'btn-danger' : 'btn-outline-danger'"
          @click="mode = 'disabled'"
        >
          Disabled
        </button>
        <button
          type="button"
          class="btn flex-fill"
          :class="mode === 'throttle' ? 'btn-success' : 'btn-outline-success'"
          @click="mode = 'throttle'"
        >
          Throttle
        </button>
        <button
          type="button"
          class="btn flex-fill"
          :class="mode === 'ik-pos' ? 'btn-success' : 'btn-outline-success'"
          @click="mode = 'ik-pos'"
        >
          IK Position
        </button>
        <button
          type="button"
          class="btn flex-fill"
          :class="mode === 'ik-vel' ? 'btn-success' : 'btn-outline-success'"
          @click="mode = 'ik-vel'"
        >
          IK Velocity
        </button>
      </div>
    </div>
  </div>
</template>


<script lang="ts" setup>
import { ref, computed, onMounted, onBeforeUnmount } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'

const websocketStore = useWebsocketStore()

const mode = ref('disabled')
const gamepadConnected = ref(false)

const controllerConnected = computed(() => gamepadConnected.value)

let interval: number | undefined = undefined

const UPDATE_HZ = 30

const keyDown = (event: { key: string }) => {
  if (event.key === ' ') {
    mode.value = 'disabled'
  }
}

onMounted(() => {
  document.addEventListener('keydown', keyDown)
  interval = window.setInterval(() => {
    const gamepads = navigator.getGamepads()
    const gamepad = gamepads.find(
      gamepad => gamepad && gamepad.id.includes('Microsoft')
    )
    gamepadConnected.value = !!gamepad
    if (!gamepad) return

    const controllerData = {
      axes: gamepad.axes,
      buttons: gamepad.buttons.map(button => button.value)
    }

    websocketStore.sendMessage('arm', {
      type: 'ra_controller',
      ...controllerData
    })

    websocketStore.sendMessage('arm', {
      type: 'ra_mode',
      mode: mode.value,
    })

    websocketStore.sendMessage('drive', {
      type: 'controller',
      ...controllerData
    })
  }, 1000 / UPDATE_HZ)
})

onBeforeUnmount(() => {
  window.clearInterval(interval)
  document.removeEventListener('keydown', keyDown)
})
</script>

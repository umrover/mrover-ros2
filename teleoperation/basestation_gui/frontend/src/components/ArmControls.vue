<template>
  <div class="d-flex flex-column align-items-center w-100">
    <div class="d-flex flex-column gap-2" style="width: 500px; max-width: 100%;">
      <div class="d-flex justify-content-between align-items-center">
        <h3 class="m-0">Arm Controls</h3>
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
          @click="newRAMode('disabled')"
        >
          Disabled
        </button>
        <button
          type="button"
          class="btn flex-fill"
          :class="mode === 'throttle' ? 'btn-success' : 'btn-outline-success'"
          @click="newRAMode('throttle')"
        >
          Throttle
        </button>
        <button
          type="button"
          class="btn flex-fill"
          :class="mode === 'ik-pos' ? 'btn-success' : 'btn-outline-success'"
          @click="newRAMode('ik-pos')"
        >
          IK Position
        </button>
        <button
          type="button"
          class="btn flex-fill"
          :class="mode === 'ik-vel' ? 'btn-success' : 'btn-outline-success'"
          @click="newRAMode('ik-vel')"
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
import { armAPI } from '@/utils/api'

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

    websocketStore.sendMessage('arm', {
      type: 'ra_controller',
      axes: gamepad.axes,
      buttons: gamepad.buttons.map(button => button.value),
    })
  }, 1000 / UPDATE_HZ)
})

onBeforeUnmount(() => {
  window.clearInterval(interval)
  document.removeEventListener('keydown', keyDown)
})

const newRAMode = async (newMode: string) => {
  try {
    mode.value = newMode
    const data = await armAPI.setRAMode(mode.value)
    if (data.status === 'success' && data.mode) {
        mode.value = data.mode
      };
    }
     catch (error) {
    console.error('Failed to set arm mode:', error)
  }
}
</script>

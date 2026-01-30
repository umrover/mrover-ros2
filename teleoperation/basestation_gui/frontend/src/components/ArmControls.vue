<template>
  <div class="d-flex flex-column align-items-center">
    <div class="d-flex flex-column gap-2 align-items-center">
      <div class="d-flex justify-content-between align-items-center w-100">
        <h4 class="m-0">Arm Controls</h4>
        <IndicatorDot :is-active="controllerConnected" class="me-2" />
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
          IK Pos
        </button>
        <button
          type="button"
          class="btn flex-fill"
          :class="mode === 'ik-vel' ? 'btn-success' : 'btn-outline-success'"
          @click="newRAMode('ik-vel')"
        >
          IK Vel
        </button>

        <button
          type="button"
          class="btn flex-fill"
          @click="doNothing()"
      >
      TEST
    </button>
      </div>
      <GamepadDisplay :axes="axes" :buttons="buttons" layout="horizontal" />
    </div>
  </div>
</template>


<script lang="ts" setup>
import { ref, computed, onMounted, onBeforeUnmount } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { armAPI } from '@/utils/api'
import GamepadDisplay from './GamepadDisplay.vue'
import IndicatorDot from './IndicatorDot.vue'

const websocketStore = useWebsocketStore()

const mode = ref('disabled')
const gamepadConnected = ref(false)
const axes = ref<number[]>([0, 0, 0, 0])
const buttons = ref<number[]>(new Array(17).fill(0))

const controllerConnected = computed(() => gamepadConnected.value)

let interval: number | undefined = undefined

const UPDATE_HZ = 30

const keyDown = async (event: { key: string }) => {
  if (event.key === ' ') {
    await newRAMode('disabled')
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

    axes.value = [...gamepad.axes]
    buttons.value = gamepad.buttons.map(button => button.value)

    const controllerData = {
      axes: gamepad.axes,
      buttons: gamepad.buttons.map(button => button.value)
    }

    websocketStore.sendMessage('arm', {
      type: 'ra_controller',
      ...controllerData
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

const doNothing = async() => {
  try {
    const data = await armAPI.stowArm()
    if (data) {
        console.log("done it")
      };
    }
     catch (error) {
    console.log('TESTING: ', error)
  }
}
</script>

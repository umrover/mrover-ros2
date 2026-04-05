<template>
  <div class="flex flex-col gap-2 h-full">
    <div class="flex justify-between items-center">
      <h4 class="component-header">Arm Controls</h4>
      <p 
      class="text-danger"
      :class="forcing_limit === true ? 'visible' : 'invisible'">
        Limit Reached!
      </p>
      <IndicatorDot :is-active="connected" class="mr-2" />
    </div>
    <div class="flex w-full" role="group" aria-label="Arm mode selection" data-testid="pw-arm-mode-buttons">
      <div class="btn-group-connected w-full">
        <button
          type="button"
          class="btn btn-sm flex-1"
          :class="mode === 'disabled' ? 'btn-danger' : 'btn-outline-danger'"
          data-testid="pw-arm-mode-disabled"
          @click="newRAMode('disabled')"
          >
          Disabled
        </button>
        <button
          type="button"
          class="btn btn-sm flex-1"
          :class="isStowing ? 'btn-warning' : 'btn-outline-warning'"
          data-testid="pw-arm-mode-stow"
          :disabled="isStowing"
          @click="stowArm"
        >
          Stow
        </button>
        <button
          type="button"
          class="btn btn-sm flex-1"
          :class="mode === 'throttle' ? 'btn-success' : 'btn-outline-success'"
          data-testid="pw-arm-mode-throttle"
          @click="newRAMode('throttle')"
        >
          Throttle
        </button>
        <button
          type="button"
          class="btn btn-sm flex-1"
          :class="mode === 'ik-pos' ? 'btn-success' : 'btn-outline-success'"
          data-testid="pw-arm-mode-ik-pos"
          @click="newRAMode('ik-pos')"
        >
          IK Pos
        </button>
        <button
          type="button"
          class="btn btn-sm flex-1"
          :class="mode === 'ik-vel' ? 'btn-success' : 'btn-outline-success'"
          data-testid="pw-arm-mode-ik-vel"
          @click="newRAMode('ik-vel')"
        >
          IK Vel
        </button>
      </div>
    </div>
    <GamepadDisplay :axes="axes" :buttons="buttons" layout="horizontal" class="grow min-h-0" />

    <div>
      <form action="/submit-data" method="POST">
        <div>
          <label for="field1">x:</label>
          <input type="number" id="x" name="x" placeholder="x">
        </div>
        
        <div>
          <label for="field2">y:</label>
          <input type="number" id="y" name="y" placeholder="y">
        </div>

        <div>
          <label for="field3">z:</label>
          <input type="number" id="z" name="z" placeholder="z">
        </div>

        <button type="submit">Submit</button>
      </form>

    </div>

  </div>

  <div>
    <p>Distance: {{stowDistance}}</p>
  </div>
</template>

<script lang="ts" setup>
import { ref, onMounted, onBeforeUnmount, computed } from 'vue'
import { armAPI } from '@/utils/api'
import { useGamepadPolling } from '@/composables/useGamepadPolling'
import { useWebsocketStore } from '@/stores/websocket'
import type { IkFeedbackMessage } from '@/types/websocket'
import GamepadDisplay from './GamepadDisplay.vue'
import IndicatorDot from './IndicatorDot.vue'

const { onMessage } = useWebsocketStore()

const mode = ref('disabled')
const forcing_limit = ref(false)

const isStowing = ref(false)
const stowTarget = ref<{ x: number; y: number; z: number } >({x: 1.124319, y: 0.0, z: 0.042229})
const position = ref<{ x: number; y: number; z: number } >({x: 0.0, y: 0.0, z: 0.0})
const stowDistance = ref(0)
const closeEnough = ref(false)

const { connected, axes, buttons } = useGamepadPolling({
  controllerIdFilter: 'Microsoft',
  topic: 'arm',
  messageType: 'ra_controller',
})

const keyDown = async (event: { key: string }) => {
  if (event.key === ' ') {
    await newRAMode('disabled')
  }
}

onMounted(() => {
  document.addEventListener('keydown', keyDown)
})

onBeforeUnmount(() => {
  document.removeEventListener('keydown', keyDown)
})

onMessage<IkFeedbackMessage>('arm', 'ik_feedback', (msg) => {
  position.value = msg.pos
  console.log('yippee:', msg.pos)
})

const stowArm = async () => {
  try {
    isStowing.value = true
    const result = await armAPI.stowArm()
    if (result.status === 'success') {
      mode.value = 'stow'
      stowTarget.value = {
        x: result.stow_target.pos.x,
        y: result.stow_target.pos.y,
        z: result.stow_target.pos.z,
      }

      let time = 0
      const interval = setInterval(() => {
        time += 100 // 0.1 s

        if (stowTarget.value) {
          stowDistance.value = euclideanDistance.value
        }

        if (stowDistance.value < 0.1) {
          closeEnough.value = true
          clearInterval(interval)
          isStowing.value = false
          // reached target
        } else if (time >= 5000) {
          console.log('stow timed out')
          clearInterval(interval)
          isStowing.value = false
          // 5s timeout
        }
      }, 100)
    } else {
      isStowing.value = false
    }
  } catch (error) {
    console.error('Failed to start stow:', error)
    isStowing.value = false
  }
}

const newRAMode = async (newMode: string) => {
  try {
    isStowing.value = false
    mode.value = newMode
    const data = await armAPI.setRAMode(mode.value)
    if (data.status === 'success' && data.mode) {
      mode.value = data.mode
    }
  } catch (error) {
    console.error('Failed to set arm mode:', error)
  }
}

const euclideanDistance = computed(() => {
  const a = position.value
  const b = stowTarget.value
  
  return Math.sqrt(
    (a.x - b.x) ** 2 + 
    (a.y - b.y) ** 2 + 
    (a.z - b.z) ** 2
  )
})
// current task: show distance, set timeout

</script>

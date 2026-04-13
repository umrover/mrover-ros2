<template>
  <div class="flex flex-col gap-2 h-full">
    <div class="flex justify-between items-center">
      <h4 class="component-header">Arm Controls</h4>
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
          :class="mode === 'stow' ? 'btn-warning' : 'btn-outline-warning'"
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

const STOW_DISTANCE_THRESHOLD_M = 0.1
const STOW_TIMEOUT_MS = 5000
const STOW_POLL_INTERVAL_MS = 100

type Point3 = { x: number; y: number; z: number }

const mode = ref('disabled')
const isStowing = ref(false)
const stowTarget = ref<Point3 | null>(null)
const position = ref<Point3>({ x: 0, y: 0, z: 0 })

let stowWatcherHandle: ReturnType<typeof setInterval> | null = null

const { connected, axes, buttons } = useGamepadPolling({
  controllerIdFilter: 'Microsoft',
  topic: 'arm',
  messageType: 'ra_controller',
})

const euclideanDistance = computed(() => {
  if (!stowTarget.value) return Infinity
  const a = position.value
  const b = stowTarget.value
  return Math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
})

const stopStowWatcher = () => {
  if (stowWatcherHandle !== null) {
    clearInterval(stowWatcherHandle)
    stowWatcherHandle = null
  }
  isStowing.value = false
}

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
  stopStowWatcher()
})

onMessage<IkFeedbackMessage>('arm', 'ik_feedback', (msg) => {
  position.value = msg.pos
})

const stowArm = async () => {
  try {
    stopStowWatcher()
    isStowing.value = true
    const result = await armAPI.stowArm()
    if (result.status !== 'success') {
      isStowing.value = false
      return
    }

    mode.value = 'stow'
    stowTarget.value = {
      x: result.stow_target.pos.x,
      y: result.stow_target.pos.y,
      z: result.stow_target.pos.z,
    }

    const startTime = Date.now()
    stowWatcherHandle = setInterval(() => {
      if (
        euclideanDistance.value < STOW_DISTANCE_THRESHOLD_M ||
        Date.now() - startTime >= STOW_TIMEOUT_MS
      ) {
        stopStowWatcher()
      }
    }, STOW_POLL_INTERVAL_MS)
  } catch (error) {
    console.error('Failed to start stow:', error)
    stopStowWatcher()
  }
}

const newRAMode = async (newMode: string) => {
  try {
    stopStowWatcher()
    mode.value = newMode
    const data = await armAPI.setRAMode(mode.value)
    if (data.status === 'success' && data.mode) {
      mode.value = data.mode
    }
  } catch (error) {
    console.error('Failed to set arm mode:', error)
  }
}
</script>

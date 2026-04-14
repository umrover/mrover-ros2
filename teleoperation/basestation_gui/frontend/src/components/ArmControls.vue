<template>
  <div class="flex flex-col gap-2 h-full">
    <div class="flex justify-between items-center">
      <div class="flex items-center gap-3">
        <h4 class="component-header">Arm Controls</h4>
        <span v-if="mode === 'stow'" class="text-sm font-mono" data-testid="pw-arm-stow-distance">
          Stow distance: {{ euclideanDistance.toFixed(2) }} m
        </span>
      </div>
      <div class="flex items-center gap-2">
        <button
          type="button"
          class="cmd-btn cmd-btn-sm cmd-btn-outline-secondary btn-icon"
          data-testid="pw-arm-stow-config"
          title="Edit stow target"
          @click="openStowModal"
        >
          <i class="bi bi-gear"></i>
        </button>
        <IndicatorDot :is-active="connected" class="mr-2" />
      </div>
    </div>
    <StowConfigModal
      v-if="stowConfig"
      :is-open="stowModalOpen"
      :initial="stowConfig"
      @close="stowModalOpen = false"
      @saved="onStowSaved"
    />
    <div class="btn-group w-full" role="group" aria-label="Arm mode selection" data-testid="pw-arm-mode-buttons">
      <button
        type="button"
        class="cmd-btn cmd-btn-sm"
        :class="mode === 'disabled' ? 'cmd-btn-danger' : 'cmd-btn-outline-danger'"
        data-testid="pw-arm-mode-disabled"
        @click="newRAMode('disabled')"
      >
        Disabled
      </button>
      <button
        type="button"
        class="cmd-btn cmd-btn-sm"
        :class="mode === 'stow' ? 'cmd-btn-warning' : 'cmd-btn-outline-warning'"
        data-testid="pw-arm-mode-stow"
        :disabled="isStowing"
        @click="stowArm"
      >
        Stow
      </button>
      <button
        type="button"
        class="cmd-btn cmd-btn-sm"
        :class="mode === 'throttle' ? 'cmd-btn-success' : 'cmd-btn-outline-success'"
        data-testid="pw-arm-mode-throttle"
        @click="newRAMode('throttle')"
      >
        Throttle
      </button>
      <button
        type="button"
        class="cmd-btn cmd-btn-sm"
        :class="mode === 'ik-pos' ? 'cmd-btn-success' : 'cmd-btn-outline-success'"
        data-testid="pw-arm-mode-ik-pos"
        @click="newRAMode('ik-pos')"
      >
        IK Pos
      </button>
      <button
        type="button"
        class="cmd-btn cmd-btn-sm"
        :class="mode === 'ik-vel' ? 'cmd-btn-success' : 'cmd-btn-outline-success'"
        data-testid="pw-arm-mode-ik-vel"
        @click="newRAMode('ik-vel')"
      >
        IK Vel
      </button>
    </div>
    <GamepadDisplay :axes="axes" :buttons="buttons" layout="horizontal" class="grow min-h-0" />
  </div>
</template>

<script lang="ts" setup>
import { ref, onMounted, onBeforeUnmount, computed } from 'vue'
import { armAPI } from '@/utils/api'
import { useGamepadPolling } from '@/composables/useGamepadPolling'
import { useWebsocketStore } from '@/stores/websocket'
import { useNotificationsStore } from '@/stores/notifications'
import type { IkFeedbackMessage } from '@/types/websocket'
import type { StowPosition } from '@/utils/apiTypes'
import GamepadDisplay from './GamepadDisplay.vue'
import IndicatorDot from './IndicatorDot.vue'
import StowConfigModal from './StowConfigModal.vue'

const { onMessage } = useWebsocketStore()
const notificationsStore = useNotificationsStore()

const STOW_DISTANCE_THRESHOLD_M = 0.1
const STOW_TIMEOUT_MS = 10000
const STOW_POLL_INTERVAL_MS = 100

type Point3 = { x: number; y: number; z: number }

const mode = ref('disabled')
const isStowing = ref(false)
const stowTarget = ref<Point3 | null>(null)
const position = ref<Point3>({ x: 0, y: 0, z: 0 })
const stowModalOpen = ref(false)
const stowConfig = ref<StowPosition | null>(null)

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

const loadStowConfig = async () => {
  try {
    const result = await armAPI.getStowConfig()
    if (result.status === 'success' && result.stow_position) {
      stowConfig.value = result.stow_position
    }
  } catch (error) {
    notificationsStore.addNotification({
      component: 'Arm Controls',
      message: 'Failed to load stow config.',
      fullData: { error: String(error) }
    })
  }
}

const openStowModal = async () => {
  await loadStowConfig()
  if (stowConfig.value) {
    stowModalOpen.value = true
  }
}

const onStowSaved = (value: StowPosition) => {
  stowConfig.value = value
}

onMounted(() => {
  document.addEventListener('keydown', keyDown)
  loadStowConfig()
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
    stowWatcherHandle = setInterval(async () => {
      if (euclideanDistance.value < STOW_DISTANCE_THRESHOLD_M) {
        stopStowWatcher()
        await newRAMode('disabled')
      } else if (Date.now() - startTime >= STOW_TIMEOUT_MS) {
        notificationsStore.addNotification({
          component: 'Arm Controls',
          message: `Stow timed out after ${STOW_TIMEOUT_MS / 1000}s. Arm did not reach target within ${STOW_DISTANCE_THRESHOLD_M}m.`,
          fullData: {
            distanceM: euclideanDistance.value,
            thresholdM: STOW_DISTANCE_THRESHOLD_M,
            timeoutMs: STOW_TIMEOUT_MS,
            target: stowTarget.value,
            position: position.value
          }
        })
        stopStowWatcher()
        await newRAMode('disabled')
      }
    }, STOW_POLL_INTERVAL_MS)
  } catch (error) {
    notificationsStore.addNotification({
      component: 'Arm Controls',
      message: 'Failed to start stow.',
      fullData: { error: String(error) }
    })
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
    notificationsStore.addNotification({
      component: 'Arm Controls',
      message: `Failed to set arm mode to '${newMode}'.`,
      fullData: { error: String(error) }
    })
  }
}
</script>

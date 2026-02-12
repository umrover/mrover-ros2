<template>
  <div class="flex flex-col items-center p-1" data-testid="pw-funnel-controls">
    <div class="flex justify-between items-center w-full mb-2">
      <h4 class="component-header">Funnel Controls</h4>
    </div>
    <div class="flex flex-col gap-1 w-full">
      <div class="flex gap-1 justify-center">
        <button
          class="cmd-btn cmd-btn-outline-control cmd-btn-sm w-[70px] py-[0.2rem]"
          :class="{ active: currentSite === 1, 'cmd-btn-warning': isLoading && pendingSite === 1 }"
          :disabled="isLoading"
          :data-testid="`pw-funnel-site-1`"
          @click="selectSite(1)"
        >
          Sample
        </button>
      </div>
      <div class="flex gap-1 justify-between">
        <button
          class="cmd-btn cmd-btn-outline-control cmd-btn-sm w-[70px] py-[0.2rem]"
          :class="{ active: currentSite === 0, 'cmd-btn-warning': isLoading && pendingSite === 0 }"
          :disabled="isLoading"
          :data-testid="`pw-funnel-site-0`"
          @click="selectSite(0)"
        >
          Griess B
        </button>
        <button
          class="cmd-btn cmd-btn-outline-control cmd-btn-sm w-[70px] py-[0.2rem]"
          :class="{ active: currentSite === 2, 'cmd-btn-warning': isLoading && pendingSite === 2 }"
          :disabled="isLoading"
          :data-testid="`pw-funnel-site-2`"
          @click="selectSite(2)"
        >
          Buret A
        </button>
      </div>
      <div class="flex gap-1 justify-between">
        <button
          class="cmd-btn cmd-btn-outline-control cmd-btn-sm w-[70px] py-[0.2rem]"
          :class="{ active: currentSite === 3, 'cmd-btn-warning': isLoading && pendingSite === 3 }"
          :disabled="isLoading"
          :data-testid="`pw-funnel-site-3`"
          @click="selectSite(3)"
        >
          Buret B
        </button>
        <button
          class="cmd-btn cmd-btn-outline-control cmd-btn-sm w-[70px] py-[0.2rem]"
          :class="{ active: currentSite === 5, 'cmd-btn-warning': isLoading && pendingSite === 5 }"
          :disabled="isLoading"
          :data-testid="`pw-funnel-site-5`"
          @click="selectSite(5)"
        >
          Griess A
        </button>
      </div>
      <div class="flex gap-1 justify-center">
        <button
          class="cmd-btn cmd-btn-outline-control cmd-btn-sm w-[70px] py-[0.2rem]"
          :class="{ active: currentSite === 4, 'cmd-btn-warning': isLoading && pendingSite === 4 }"
          :disabled="isLoading"
          :data-testid="`pw-funnel-site-4`"
          @click="selectSite(4)"
        >
          Trash
        </button>
      </div>
    </div>
    <div class="flex flex-col gap-1 w-full mt-2 pt-2 border-t border-gray-600">
      <div class="flex justify-between items-center">
        <span class="text-xs font-semibold uppercase text-gray-400">Fine Adjust</span>
        <span class="text-sm font-mono font-semibold px-2 py-0.5 rounded bg-[var(--card-bg)] border-2 border-[var(--input-border)]">
          {{ currentDegrees }}&deg;
        </span>
      </div>
      <div class="flex gap-1">
        <button class="cmd-btn cmd-btn-outline-control cmd-btn-sm flex-1" :disabled="!hasFunnelState" @click="adjustFunnel(-10)">-10</button>
        <button class="cmd-btn cmd-btn-outline-control cmd-btn-sm flex-1" :disabled="!hasFunnelState" @click="adjustFunnel(-5)">-5</button>
        <button class="cmd-btn cmd-btn-outline-control cmd-btn-sm flex-1" :disabled="!hasFunnelState" @click="adjustFunnel(-1)">-1</button>
        <button class="cmd-btn cmd-btn-outline-control cmd-btn-sm flex-1" :disabled="!hasFunnelState" @click="adjustFunnel(1)">+1</button>
        <button class="cmd-btn cmd-btn-outline-control cmd-btn-sm flex-1" :disabled="!hasFunnelState" @click="adjustFunnel(5)">+5</button>
        <button class="cmd-btn cmd-btn-outline-control cmd-btn-sm flex-1" :disabled="!hasFunnelState" @click="adjustFunnel(10)">+10</button>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed } from 'vue'
import { scienceAPI } from '@/utils/api'
import { useWebsocketStore } from '@/stores/websocket'
import type { ControllerStateMessage } from '@/types/websocket'

const site_to_radians: Record<number, number> = {
  0: 0.0,
  1: Math.PI / 3,
  2: (2 * Math.PI) / 3,
  3: Math.PI,
  4: (4 * Math.PI) / 3,
  5: (5 * Math.PI) / 3,
}

const websocketStore = useWebsocketStore()

// TODO(funnel): Read the funnel motor position from WebSocket messages.
//
// The backend (science_ws.py) forwards '/sp_controller_state' messages
// over the 'science' WebSocket with type 'sp_controller_state'.
//
// Steps:
//   1. Get the science messages: websocketStore.messages['science']
//   2. Check if the message exists and has type === 'sp_controller_state'
//   3. Cast it to ControllerStateMessage and return it
//   4. Return null if no matching message found
//
// Hint: Look at GimbalControls.vue -- it does the exact same thing
// but reads from 'chassis' messages with type 'gimbal_controller_state'.
const funnelState = computed((): ControllerStateMessage | null => {
  // TODO(funnel): Implement this
  return null
})

const hasFunnelState = computed((): boolean => {
  return funnelState.value !== null
})

// TODO(funnel): Extract the funnel position in degrees from funnelState.
//
// Steps:
//   1. Get funnelState.value (return '0' if null)
//   2. Find the index of 'funnel' in state.names array
//   3. Get the position at that index from state.positions (in radians)
//   4. Convert to degrees: (radians * 180) / Math.PI
//   5. Return as a string with .toFixed(0)
//
// Hint: See pitchDegrees/yawDegrees in GimbalControls.vue.
const currentDegrees = computed((): string => {
  // TODO(funnel): Implement this
  return '0'
})

// TODO(funnel): Send a micro-adjustment to the funnel.
//
// Steps:
//   1. Get the current position in radians from funnelState
//      (same lookup as currentDegrees but don't convert)
//   2. Convert the degrees parameter to radians: (degrees * Math.PI) / 180
//   3. Add the adjustment to the current position
//   4. Call: await scienceAPI.setGearDiffPosition(newPosition, false)
//
// Hint: Same pattern as adjustGimbal() in GimbalControls.vue,
// but using scienceAPI instead of chassisAPI.
async function adjustFunnel(_degrees: number) {
  // TODO(funnel): Implement this
}

const currentSite = ref(0)
const pendingSite = ref<number | null>(null)
const isLoading = ref(false)

async function selectSite(index: number) {
  if (index === currentSite.value || isLoading.value) return

  const previousSite = currentSite.value
  currentSite.value = index
  pendingSite.value = index
  isLoading.value = true

  try {
    const radians = site_to_radians[index]
    if (radians !== undefined) {
      await scienceAPI.setGearDiffPosition(radians, false)
    }
  } catch {
    currentSite.value = previousSite
  } finally {
    isLoading.value = false
    pendingSite.value = null
  }
}
</script>


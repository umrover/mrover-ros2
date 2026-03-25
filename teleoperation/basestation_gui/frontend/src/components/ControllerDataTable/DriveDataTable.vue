<template>
  <div class="flex flex-col gap-2 h-full">
    <h4 class="component-header">Drive</h4>
    <div class="overflow-x-auto cmd-scroll flex-1">
      <table class="cmd-table compact-table w-full">
        <thead>
          <tr>
            <th>Wheel</th>
            <th>State</th>
            <th>Err</th>
            <th>Vel</th>
            <th>Cur</th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="w in wheels" :key="w.id" :class="stale ? 'row-no-data' : stateRowClass(combined.states[w.idx], combined.errors[w.idx])">
            <td class="font-bold">{{ w.label }}</td>
            <td>{{ formatState(combined.states[w.idx]) }}</td>
            <td>{{ formatError(combined.errors[w.idx]) }}</td>
            <td class="numeric-col">{{ formatNumber(combined.velocities[w.idx]) }}</td>
            <td class="numeric-col">{{ formatNumber(combined.currents[w.idx]) }}</td>
          </tr>
        </tbody>
      </table>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import type { ControllerStateMessage } from '@/types/websocket'
import { useStaleTimer, formatState, formatNumber, formatError, stateRowClass } from '@/composables/useControllerState'

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)
const { stale, reset: resetStale } = useStaleTimer()

let leftState: ControllerStateMessage | null = null
let rightState: ControllerStateMessage | null = null

const combined = ref({
  states: [] as string[],
  errors: [] as string[],
  velocities: [] as number[],
  currents: [] as number[],
})

function mergeLeftRight() {
  const l = leftState ?? { states: [], errors: [], velocities: [], currents: [] }
  const r = rightState ?? { states: [], errors: [], velocities: [], currents: [] }
  combined.value = {
    states: [...l.states, ...r.states],
    errors: [...l.errors, ...r.errors],
    velocities: [...l.velocities, ...r.velocities],
    currents: [...l.currents, ...r.currents],
  }
}

const driveMessage = computed(() => messages.value['drive'])

watch(driveMessage, (msg) => {
  if (!msg) return
  const typed = msg as ControllerStateMessage
  if (typed.type === 'drive_left_state') {
    resetStale()
    leftState = typed
    mergeLeftRight()
  } else if (typed.type === 'drive_right_state') {
    resetStale()
    rightState = typed
    mergeLeftRight()
  }
})

const wheels = [
  { id: 'fl', label: 'Front L', idx: 0 },
  { id: 'fr', label: 'Front R', idx: 3 },
  { id: 'ml', label: 'Mid L',   idx: 1 },
  { id: 'mr', label: 'Mid R',   idx: 4 },
  { id: 'rl', label: 'Rear L',  idx: 2 },
  { id: 'rr', label: 'Rear R',  idx: 5 },
]
</script>

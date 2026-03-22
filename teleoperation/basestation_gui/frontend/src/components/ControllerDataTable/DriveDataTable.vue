<template>
  <div class="flex flex-col gap-2 h-full">
    <h4 class="component-header">Drive</h4>
    <div class="overflow-x-auto scroll flex-1">
      <table class="table compact-table w-full">
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
          <tr v-for="w in wheels" :key="w.id" :class="stale ? 'row-no-data' : stateRowClass(states[w.idx], errors[w.idx])">
            <td class="font-bold">{{ w.label }}</td>
            <td>{{ formatState(states[w.idx]) }}</td>
            <td>{{ formatError(errors[w.idx]) }}</td>
            <td v-html="formatNumber(velocities[w.idx], 1, 2, true)"></td>
            <td v-html="formatNumber(currents[w.idx], 1)"></td>
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
import { useStaleTimer, formatState, formatError, stateRowClass } from '@/composables/useControllerState'
import { formatNumber } from '@/utils/formatNumber'

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)
const { stale, reset: resetStale } = useStaleTimer()

const states = ref<string[]>([])
const errors = ref<string[]>([])
const velocities = ref<number[]>([])
const currents = ref<number[]>([])

let leftState: ControllerStateMessage | null = null
let rightState: ControllerStateMessage | null = null

function combineLeftRight() {
  const l = leftState || { states: [], errors: [], velocities: [], currents: [] }
  const r = rightState || { states: [], errors: [], velocities: [], currents: [] }

  states.value = [...l.states, ...r.states]
  errors.value = [...l.errors, ...r.errors]
  velocities.value = [...l.velocities, ...r.velocities]
  currents.value = [...l.currents, ...r.currents]
}

const driveMessage = computed(() => messages.value['drive'])

watch(driveMessage, (msg) => {
  if (!msg) return
  const typed = msg as ControllerStateMessage
  if (typed.type === 'drive_left_state') {
    resetStale()
    leftState = typed
    combineLeftRight()
  } else if (typed.type === 'drive_right_state') {
    resetStale()
    rightState = typed
    combineLeftRight()
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

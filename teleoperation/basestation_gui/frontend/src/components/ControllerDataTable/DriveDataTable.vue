<template>
<<<<<<< HEAD
  <div class="flex flex-col gap-2 h-full">
    <h4 class="component-header">Drive</h4>
    <div class="overflow-x-auto cmd-scroll flex-1">
      <table class="cmd-table compact-table w-full">
=======
  <div class="flex flex-col gap-1 h-full">
    <div class="flex justify-between items-center pr-2 py-0">
      <h4 class="component-header">Drive State</h4>
      <button class="btn btn-icon-sm !h-6 !w-6 btn-outline-info" @click="legendModal?.open()">
        <i class="bi bi-info-circle"></i>
      </button>
    </div>
    <div class="overflow-x-auto scroll flex-1">
      <table class="table compact-table w-full">
>>>>>>> origin/main
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
<<<<<<< HEAD
          <tr v-for="w in wheels" :key="w.id" :class="stale ? 'row-no-data' : stateRowClass(states[w.idx], errors[w.idx])">
            <td class="font-bold">{{ w.label }}</td>
            <td>{{ formatState(states[w.idx]) }}</td>
            <td>{{ formatError(errors[w.idx]) }}</td>
            <td class="numeric-col">{{ formatNumber(velocities[w.idx]) }}</td>
            <td class="numeric-col">{{ formatNumber(currents[w.idx]) }}</td>
=======
          <tr v-for="w in wheels" :key="w.id" :class="stale ? 'row-no-data' : stateRowClass(stateFor(w.id), errorFor(w.id))">
            <td class="font-bold">{{ w.label }}</td>
            <td>{{ formatState(stateFor(w.id)) }}</td>
            <td>{{ formatError(errorFor(w.id)) }}</td>
            <td class="numeric-col"><span v-html="formatNumber(fieldAt(data.velocities, w.id), 3, 2, true)"></span></td>
            <td class="numeric-col"><span v-html="formatNumber(fieldAt(data.currents, w.id), 3, 2, true)"></span></td>
>>>>>>> origin/main
          </tr>
        </tbody>
      </table>
    </div>
<<<<<<< HEAD
=======
    <StateMappingModal ref="legendModal" />
>>>>>>> origin/main
  </div>
</template>

<script lang="ts" setup>
<<<<<<< HEAD
import { ref, computed, watch } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import type { ControllerStateMessage } from '@/types/websocket'
import { useStaleTimer, formatState, formatNumber, formatError, stateRowClass } from '@/composables/useControllerState'

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
=======
import { ref } from 'vue'
import { useControllerMessage, formatState, formatNumber, formatError, stateRowClass } from '@/composables/useControllerState'
import StateMappingModal from '@/components/StateMappingModal.vue'

const legendModal = ref<InstanceType<typeof StateMappingModal> | null>(null)

const { stale, data } = useControllerMessage({
  topic: 'drive',
  messageType: 'drive_state',
})

const wheels = [
  { id: 'front_left', label: 'Front L' },
  { id: 'front_right', label: 'Front R' },
  { id: 'middle_left', label: 'Mid L' },
  { id: 'middle_right', label: 'Mid R' },
  { id: 'back_left', label: 'Rear L' },
  { id: 'back_right', label: 'Rear R' },
]

function indexFor(id: string): number {
  return data.value.names.indexOf(id)
}

function fieldAt<T>(arr: T[], id: string): T | undefined {
  const i = indexFor(id)
  return i >= 0 && i < arr.length ? arr[i] : undefined
}

function stateFor(id: string): string | undefined {
  return fieldAt(data.value.states, id)
}

function errorFor(id: string): string | undefined {
  return fieldAt(data.value.errors, id)
}
>>>>>>> origin/main
</script>

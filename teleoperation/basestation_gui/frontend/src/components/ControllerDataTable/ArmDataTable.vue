<template>
<<<<<<< HEAD
  <div class="flex flex-col gap-2 h-full">
    <h4 class="component-header">Arm</h4>
    <div class="overflow-x-auto cmd-scroll flex-1">
      <table class="cmd-table compact-table w-full">
=======
  <div class="flex flex-col gap-1 h-full">
    <div class="flex justify-between items-center pr-2 py-0">
      <h4 class="component-header">Arm State</h4>
      <button class="btn btn-icon-sm !h-6 !w-6 btn-outline-info" @click="legendModal?.open()">
        <i class="bi bi-info-circle"></i>
      </button>
    </div>
    <div class="overflow-x-auto scroll flex-1">
      <table class="table compact-table w-full">
>>>>>>> origin/main
        <thead>
          <tr>
            <th>Joint</th>
            <th>State</th>
            <th>Err</th>
            <th>Pos</th>
            <th>Vel</th>
            <th>Cur</th>
            <th>Lim</th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="j in joints" :key="j.id" :class="stale ? 'row-no-data' : stateRowClass(stateFor(j.id), errorFor(j.id))">
            <td class="font-bold">{{ j.label }}</td>
            <td>{{ formatState(stateFor(j.id)) }}</td>
            <td>{{ formatError(errorFor(j.id)) }}</td>
<<<<<<< HEAD
            <td class="numeric-col">{{ formatNumber(valueFor(positions, j.id)) }}</td>
            <td class="numeric-col">{{ formatNumber(valueFor(velocities, j.id)) }}</td>
            <td class="numeric-col">{{ formatNumber(valueFor(currents, j.id)) }}</td>
            <td>{{ formatLimit(limitFor(j.id)) }}</td>
=======
            <td class="numeric-col"><span v-html="formatNumber(fieldAt(data.positions, j.id), 3, 2, true)"></span></td>
            <td class="numeric-col"><span v-html="formatNumber(fieldAt(data.velocities, j.id), 3, 2, true)"></span></td>
            <td class="numeric-col"><span v-html="formatNumber(fieldAt(data.currents, j.id), 3, 2, true)"></span></td>
            <td>{{ formatLimit(fieldAt(data.limitsHit, j.id)) }}</td>
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
import { useStaleTimer, formatState, formatNumber, formatLimit, formatError, stateRowClass } from '@/composables/useControllerState'

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)
const { stale, reset: resetStale } = useStaleTimer()

const names = ref<string[]>([])
const states = ref<string[]>([])
const errors = ref<string[]>([])
const positions = ref<number[]>([])
const velocities = ref<number[]>([])
const currents = ref<number[]>([])
const limitHits = ref<number[]>([])

const armMessage = computed(() => messages.value['arm'])

watch(armMessage, (msg) => {
  if (!msg) return
  const typed = msg as ControllerStateMessage
  if (typed.type !== 'arm_state') return
  resetStale()
  names.value = typed.names
  states.value = typed.states
  errors.value = typed.errors
  positions.value = typed.positions
  velocities.value = typed.velocities
  currents.value = typed.currents
  limitHits.value = typed.limits_hit
=======
import { ref } from 'vue'
import { useControllerMessage, formatState, formatNumber, formatLimit, formatError, stateRowClass } from '@/composables/useControllerState'
import StateMappingModal from '@/components/StateMappingModal.vue'

const legendModal = ref<InstanceType<typeof StateMappingModal> | null>(null)

const { stale, data } = useControllerMessage({
  topic: 'arm',
  messageType: 'arm_state',
>>>>>>> origin/main
})

const joints = [
  { id: 'gripper', label: 'Gripper' },
  { id: 'joint_de_roll', label: 'DE Roll' },
  { id: 'joint_de_pitch', label: 'DE Pitch' },
  { id: 'joint_c', label: 'Joint C' },
  { id: 'joint_b', label: 'Joint B' },
  { id: 'joint_a', label: 'Joint A' },
]

function indexFor(id: string): number {
<<<<<<< HEAD
  return names.value.indexOf(id)
=======
  return data.value.names.indexOf(id)
>>>>>>> origin/main
}

function fieldAt<T>(arr: T[], id: string): T | undefined {
  const i = indexFor(id)
  return i >= 0 && i < arr.length ? arr[i] : undefined
}

function stateFor(id: string): string | undefined {
<<<<<<< HEAD
  return fieldAt(states.value, id)
}

function errorFor(id: string): string | undefined {
  return fieldAt(errors.value, id)
}

function valueFor(arr: number[], id: string): number | undefined {
  return fieldAt(arr, id)
}

function limitFor(id: string): number | undefined {
  return fieldAt(limitHits.value, id)
=======
  return fieldAt(data.value.states, id)
}

function errorFor(id: string): string | undefined {
  return fieldAt(data.value.errors, id)
>>>>>>> origin/main
}
</script>

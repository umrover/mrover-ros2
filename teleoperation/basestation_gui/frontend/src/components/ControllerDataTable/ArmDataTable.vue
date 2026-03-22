<template>
  <div class="flex flex-col gap-2 h-full">
    <h4 class="component-header">Arm</h4>
    <div class="overflow-x-auto cmd-scroll flex-1">
      <table class="cmd-table compact-table w-full">
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
          <tr v-for="j in joints" :key="j.id" :class="stale ? 'row-no-data' : stateRowClass(fieldAt(states, j.id), fieldAt(errors, j.id))">
            <td class="font-bold">{{ j.label }}</td>
            <td>{{ formatState(fieldAt(states, j.id)) }}</td>
            <td>{{ formatError(fieldAt(errors, j.id)) }}</td>
            <td v-html="formatNumber(fieldAt(positions, j.id), 1, 2, true)"></td>
            <td v-html="formatNumber(fieldAt(velocities, j.id), 1, 2, true)"></td>
            <td v-html="formatNumber(fieldAt(currents, j.id), 1)"></td>
            <td>{{ formatLimit(fieldAt(limitHits, j.id)) }}</td>
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
import { useStaleTimer, formatState, formatLimit, formatError, stateRowClass } from '@/composables/useControllerState'
import { formatNumber } from '@/utils/formatNumber'

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
})

const joints = [
  { id: 'pusher', label: 'Pusher' },
  { id: 'gripper', label: 'Gripper' },
  { id: 'joint_de_roll', label: 'DE Roll' },
  { id: 'joint_de_pitch', label: 'DE Pitch' },
  { id: 'joint_c', label: 'Joint C' },
  { id: 'joint_b', label: 'Joint B' },
  { id: 'joint_a', label: 'Joint A' },
]

function fieldAt<T>(arr: T[], id: string): T | undefined {
  const i = names.value.indexOf(id)
  return i >= 0 && i < arr.length ? arr[i] : undefined
}
</script>

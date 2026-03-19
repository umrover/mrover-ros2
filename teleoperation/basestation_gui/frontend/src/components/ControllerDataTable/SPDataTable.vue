<template>
  <div class="flex flex-col gap-2 h-full">
    <h4 class="component-header">SP</h4>
    <div class="overflow-x-auto cmd-scroll flex-1">
      <table class="cmd-table compact-table w-full">
        <thead>
          <tr>
            <th>Motor</th>
            <th>State</th>
            <th>Err</th>
            <th>Pos</th>
            <th>Cur</th>
            <th>Lim</th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="(name, i) in names" :key="i" :class="stale ? 'row-no-data' : stateRowClass(states[i], errors[i])">
            <td class="font-bold">{{ name }}</td>
            <td>{{ formatState(states[i]) }}</td>
            <td>{{ formatError(errors[i]) }}</td>
            <td class="numeric-col">{{ formatNumber(positions[i]) }}</td>
            <td class="numeric-col">{{ formatNumber(currents[i]) }}</td>
            <td>{{ formatLimit(limitHits[i]) }}</td>
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
import { useStaleTimer, formatState, formatNumber, formatLimit, formatError, stateRowClass } from '@/composables/useControllerState'

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)
const { stale, reset: resetStale } = useStaleTimer()

const names = ref<string[]>([])
const states = ref<string[]>([])
const errors = ref<string[]>([])
const positions = ref<number[]>([])
const currents = ref<number[]>([])
const limitHits = ref<number[]>([])

const scienceMessage = computed(() => messages.value['science'])

watch(scienceMessage, (msg) => {
  if (!msg) return
  const typed = msg as ControllerStateMessage
  if (typed.type !== 'sp_controller_state') return
  resetStale()
  names.value = typed.names
  states.value = typed.states
  errors.value = typed.errors
  positions.value = typed.positions
  currents.value = typed.currents
  limitHits.value = typed.limits_hit
})
</script>

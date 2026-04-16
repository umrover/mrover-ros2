<template>
  <div class="cmd-panel controller-table-panel">
    <div class="cmd-panel-header">
      <h4>{{ header }}</h4>
      <div class="flex gap-1">
        <button
          type="button"
          class="cmd-btn cmd-btn-sm"
          :class="showStatus ? 'cmd-btn-success' : 'cmd-btn-outline-secondary'"
          data-testid="pw-controller-status-toggle"
          @mousedown.prevent
          @click="showStatus = !showStatus"
        >Status</button>
        <button
          type="button"
          class="cmd-btn cmd-btn-sm"
          :class="showValues ? 'cmd-btn-success' : 'cmd-btn-outline-secondary'"
          data-testid="pw-controller-values-toggle"
          @mousedown.prevent
          @click="showValues = !showValues"
        >Values</button>
      </div>
    </div>
    <div class="overflow-x-auto cmd-scroll">
      <table class="cmd-table compact-table">
        <tbody>
        <tr>
          <th class="sticky-col">Motor</th>
          <td v-for="(n, i) in names" :key="i">{{ n }}</td>
        </tr>
        <tr v-if="showStatus">
          <th class="sticky-col">State</th>
          <td v-for="(s, i) in states" :key="i">{{ s }}</td>
        </tr>
        <tr v-if="showStatus">
          <th class="sticky-col">Error</th>
          <td v-for="(e, i) in errors" :key="i">{{ e }}</td>
        </tr>
        <tr v-if="showStatus">
          <th class="sticky-col">Limit Hit</th>
          <td v-for="(l, i) in limitHits" :key="i">{{ l }}</td>
        </tr>
        <tr v-if="showValues">
          <th class="sticky-col">Position</th>
          <td v-for="(p, i) in positions" :key="i">{{ p.toFixed(2) }}</td>
        </tr>
        <tr v-if="showValues">
          <th class="sticky-col">Velocity</th>
          <td v-for="(v, i) in velocities" :key="i">{{ v.toFixed(2) }}</td>
        </tr>
        <tr v-if="showValues">
          <th class="sticky-col">Current</th>
          <td v-for="(c, i) in currents" :key="i">{{ c.toFixed(2) }}</td>
        </tr>
        </tbody>
      </table>
    </div>
  </div>
</template>

<script lang='ts' setup>
import { ref, watch } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import type { ControllerStateMessage } from '@/types/websocket'

const props = defineProps<{
  header: string
  mode: 'drive' | 'arm' | 'sp'
}>()

const websocketStore = useWebsocketStore()

const statusKey = `controllerDataTable_${props.mode}_showStatus`
const valuesKey = `controllerDataTable_${props.mode}_showValues`

const showStatus = ref(localStorage.getItem(statusKey) === 'true')
const showValues = ref(localStorage.getItem(valuesKey) === 'true')

watch(showStatus, (val) => localStorage.setItem(statusKey, String(val)))
watch(showValues, (val) => localStorage.setItem(valuesKey, String(val)))

const names = ref<string[]>([])
const states = ref<string[]>([])
const errors = ref<string[]>([])
const limitHits = ref<number[]>([])
const positions = ref<number[]>([])
const velocities = ref<number[]>([])
const currents = ref<number[]>([])

const leftState = ref<ControllerStateMessage | null>(null)
const rightState = ref<ControllerStateMessage | null>(null)

function updateFromMessage(msg: ControllerStateMessage) {
  if (!msg || !Array.isArray(msg.names)) return
  names.value = msg.names
  states.value = msg.states
  errors.value = msg.errors
  limitHits.value = msg.limits_hit
  positions.value = msg.positions
  velocities.value = msg.velocities
  currents.value = msg.currents
}

function combineLeftRight() {
  const left = leftState.value
  const right = rightState.value
  if (!left && !right) return

  const arr = (v: unknown): unknown[] => Array.isArray(v) ? v : []

  names.value = [...arr(left?.names), ...arr(right?.names)] as string[]
  states.value = [...arr(left?.states), ...arr(right?.states)] as string[]
  errors.value = [...arr(left?.errors), ...arr(right?.errors)] as string[]
  limitHits.value = [...arr(left?.limits_hit), ...arr(right?.limits_hit)] as number[]
  positions.value = [...arr(left?.positions), ...arr(right?.positions)] as number[]
  velocities.value = [...arr(left?.velocities), ...arr(right?.velocities)] as number[]
  currents.value = [...arr(left?.currents), ...arr(right?.currents)] as number[]
}

if (props.mode === 'drive') {
  websocketStore.onMessage<ControllerStateMessage>('drive', 'drive_left_state', (msg) => {
    leftState.value = msg
    combineLeftRight()
  })
  websocketStore.onMessage<ControllerStateMessage>('drive', 'drive_right_state', (msg) => {
    rightState.value = msg
    combineLeftRight()
  })
}

if (props.mode === 'arm') {
  websocketStore.onMessage<ControllerStateMessage>('arm', 'arm_state', (msg) => {
    updateFromMessage(msg)
  })
}

if (props.mode === 'sp') {
  websocketStore.onMessage<ControllerStateMessage>('science', 'sp_controller_state', (msg) => {
    updateFromMessage(msg)
  })
}
</script>

<style scoped>
.controller-table-panel {
  flex: 1 0 auto;
  min-width: 0;
}

.sticky-col {
  position: sticky;
  left: 0;
  z-index: 1;
  background-color: var(--table-header-bg);
}

.compact-table {
  table-layout: auto;
}

.compact-table th,
.compact-table td {
  width: 1%;
  text-align: center;
  white-space: nowrap;
}
</style>

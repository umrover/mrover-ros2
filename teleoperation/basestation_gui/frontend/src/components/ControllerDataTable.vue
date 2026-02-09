<template>
  <div class="cmd-panel controller-table-panel">
    <div class="cmd-panel-header">
      <h4>{{ header }}</h4>
      <div class="d-flex gap-1">
        <button
          type="button"
          class="btn btn-sm border-2"
          :class="showStatus ? 'btn-success' : 'btn-outline-secondary'"
          data-testid="pw-controller-status-toggle"
          @mousedown.prevent
          @click="showStatus = !showStatus"
        >Status</button>
        <button
          type="button"
          class="btn btn-sm border-2"
          :class="showValues ? 'btn-success' : 'btn-outline-secondary'"
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
import { ref, computed, watch } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import type { ControllerStateMessage } from '@/types/websocket'

const props = defineProps<{
  header: string
  mode: 'drive' | 'arm' | 'sp'
}>()

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const statusKey = `controllerDataTable_${props.mode}_showStatus`
const valuesKey = `controllerDataTable_${props.mode}_showValues`

const showStatus = ref(localStorage.getItem(statusKey) === 'true')
const showValues = ref(localStorage.getItem(valuesKey) === 'true')

watch(showStatus, (val) => localStorage.setItem(statusKey, String(val)))
watch(showValues, (val) => localStorage.setItem(valuesKey, String(val)))

const names = ref<string[]>([])
const states = ref<string[]>([])
const errors = ref<string[]>([])
const limitHits = ref<boolean[]>([])
const positions = ref<number[]>([])
const velocities = ref<number[]>([])
const currents = ref<number[]>([])

const leftState = ref<ControllerStateMessage | null>(null)
const rightState = ref<ControllerStateMessage | null>(null)

function updateFromMessage(msg: ControllerStateMessage) {
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

  const empty = { names: [], states: [], errors: [], limits_hit: [], positions: [], velocities: [], currents: [] }
  const l = left ? { ...empty, ...left } : empty
  const r = right ? { ...empty, ...right } : empty

  names.value = l.names.concat(r.names)
  states.value = l.states.concat(r.states)
  errors.value = l.errors.concat(r.errors)
  limitHits.value = l.limits_hit.concat(r.limits_hit)
  positions.value = l.positions.concat(r.positions)
  velocities.value = l.velocities.concat(r.velocities)
  currents.value = l.currents.concat(r.currents)
}

const driveMessage = computed(() => messages.value['drive'])
const armMessage = computed(() => messages.value['arm'])
const scienceMessage = computed(() => messages.value['science'])

watch(driveMessage, (msg) => {
  if (props.mode !== 'drive' || !msg) return
  const typed = msg as ControllerStateMessage
  if (typed.type === 'drive_left_state') {
    leftState.value = typed
    combineLeftRight()
  } else if (typed.type === 'drive_right_state') {
    rightState.value = typed
    combineLeftRight()
  }
})

watch(armMessage, (msg) => {
  if (props.mode !== 'arm' || !msg) return
  const typed = msg as ControllerStateMessage
  if (typed.type === 'arm_state') {
    updateFromMessage(typed)
  }
})

watch(scienceMessage, (msg) => {
  if (props.mode !== 'sp' || !msg) return
  const typed = msg as ControllerStateMessage
  if (typed.type === 'sp_controller_state') {
    updateFromMessage(typed)
  }
})
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

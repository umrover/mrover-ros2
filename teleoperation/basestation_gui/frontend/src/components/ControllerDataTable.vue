<template>
  <div class='d-flex flex-column border border-2 p-2 rounded mw-100' style="flex: 1 0 auto; min-width: 0;">
    <h4 class="mb-2">{{ header }}</h4>
    <div class="overflow-x-auto">
      <table class='table table-bordered table-sm m-0 w-auto text-nowrap compact-table'>
        <tbody>
        <tr>
          <th class='table-secondary text-nowrap sticky-col px-2 py-1'>Motor</th>
          <td v-for='(n, i) in names' :key='i' class="text-center small px-2 py-1">
            {{ n }}
          </td>
        </tr>
        <tr>
          <th class='table-secondary text-nowrap sticky-col px-2 py-1'>State</th>
          <td v-for='(s, i) in states' :key='i' class="text-center small px-2 py-1">
            {{ s }}
          </td>
        </tr>
        <tr>
          <th class='table-secondary text-nowrap sticky-col px-2 py-1'>Error</th>
          <td v-for='(e, i) in errors' :key='i' class="text-center small px-2 py-1">
            {{ e }}
          </td>
        </tr>
        <tr>
          <th class='table-secondary text-nowrap sticky-col px-2 py-1'>Limit Hit</th>
          <td v-for='(l, i) in limitHits' :key='i' class="text-center small px-2 py-1">
            {{ l }}
          </td>
        </tr>
        <tr>
          <th class='table-secondary text-nowrap sticky-col px-2 py-1'>Position</th>
          <td v-for='(p, i) in positions' :key='i' class="text-center small px-2 py-1">
            {{ p.toFixed(2) }}
          </td>
        </tr>
        <tr>
          <th class='table-secondary text-nowrap sticky-col px-2 py-1'>Velocity</th>
          <td v-for='(v, i) in velocities' :key='i' class="text-center small px-2 py-1">
            {{ v.toFixed(2) }}
          </td>
        </tr>
        <tr>
          <th class='table-secondary text-nowrap sticky-col px-2 py-1'>Current</th>
          <td v-for='(c, i) in currents' :key='i' class="text-center small px-2 py-1">
            {{ c.toFixed(2) }}
          </td>
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
  names.value = msg.name
  states.value = msg.state
  errors.value = msg.error
  limitHits.value = msg.limit_hit
  positions.value = msg.position
  velocities.value = msg.velocity
  currents.value = msg.current
}

function combineLeftRight() {
  const left = leftState.value
  const right = rightState.value
  if (!left && !right) return

  const l = left || { name: [], state: [], error: [], limit_hit: [], position: [], velocity: [], current: [] }
  const r = right || { name: [], state: [], error: [], limit_hit: [], position: [], velocity: [], current: [] }

  names.value = [...l.name, ...r.name]
  states.value = [...l.state, ...r.state]
  errors.value = [...l.error, ...r.error]
  limitHits.value = [...l.limit_hit, ...r.limit_hit]
  positions.value = [...l.position, ...r.position]
  velocities.value = [...l.velocity, ...r.velocity]
  currents.value = [...l.current, ...r.current]
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
.sticky-col {
  position: sticky;
  left: 0;
  z-index: 1;
}

.compact-table {
  table-layout: auto;
}

.compact-table th,
.compact-table td {
  white-space: nowrap;
  width: 1%;
}
</style>

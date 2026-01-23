<template>
  <div class='d-flex flex-column border border-2 p-2 rounded mw-100' style="flex: 1 0 auto; min-width: 0;">
    <h4 class="mb-2">{{ header }}</h4>
    <div class="overflow-x-auto">
      <table class='table table-bordered table-sm m-0 w-auto text-nowrap compact-table'>
        <tbody>
        <tr>
          <th class='table-secondary text-nowrap sticky-col px-2 py-1'>Motor</th>
          <td v-for='(name, i) in name' :key='i' class="text-center small px-2 py-1">
            {{ name }}
          </td>
        </tr>
        <tr>
          <th class='table-secondary text-nowrap sticky-col px-2 py-1'>State</th>
          <td v-for='(state, i) in state' :key='i' class="text-center small px-2 py-1">
            {{ state }}
          </td>
        </tr>
        <tr>
          <th class='table-secondary text-nowrap sticky-col px-2 py-1'>Error</th>
          <td v-for='(error, i) in error' :key='i' class="text-center small px-2 py-1">
            {{ error }}
          </td>
        </tr>
        <tr>
          <th class='table-secondary text-nowrap sticky-col px-2 py-1'>Limit Hit</th>
          <td v-for='(limits, i) in limits' :key='i' class="text-center small px-2 py-1">
            {{ limits }}
          </td>
        </tr>
        <tr>
          <th class='table-secondary text-nowrap sticky-col px-2 py-1'>Position</th>
          <td v-for='(pos, i) in position' :key='i' class="text-center small px-2 py-1">
            {{ pos.toFixed(2) }}
          </td>
        </tr>
        <tr>
          <th class='table-secondary text-nowrap sticky-col px-2 py-1'>Velocity</th>
          <td v-for='(vel, i) in velocity' :key='i' class="text-center small px-2 py-1">
            {{ vel.toFixed(2) }}
          </td>
        </tr>
        <tr>
          <th class='table-secondary text-nowrap sticky-col px-2 py-1'>Current</th>
          <td v-for='(cur, i) in current' :key='i' class="text-center small px-2 py-1">
            {{ cur.toFixed(2) }}
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

const props = defineProps({
  header: {
    type: String,
    required: true,
  },
  msgType: {
    type: String,
    required: true,
  }
})

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const name = ref<string[]>([])
const state = ref<string[]>([])
const error = ref<string[]>([])
const limits = ref<boolean[]>([])
const position = ref<number[]>([])
const velocity = ref<number[]>([])
const current = ref<number[]>([])

const armMessage = computed(() => messages.value['arm'])
const driveMessage = computed(() => messages.value['drive'])
const scienceMessage = computed(() => messages.value['science'])

watch(armMessage, (msg) => {
  if (msg && (msg as ControllerStateMessage).type == props.msgType) {
    const controllerMsg = msg as ControllerStateMessage;
    name.value = controllerMsg.name
    state.value = controllerMsg.state
    error.value = controllerMsg.error
    limits.value = controllerMsg.limit_hit
    position.value = controllerMsg.position
    velocity.value = controllerMsg.velocity
    current.value = controllerMsg.current
  }
})

watch(driveMessage, (msg) => {
  if (msg && (msg as ControllerStateMessage).type == props.msgType) {
    const controllerMsg = msg as ControllerStateMessage;
    name.value = controllerMsg.name
    state.value = controllerMsg.state
    error.value = controllerMsg.error
    limits.value = controllerMsg.limit_hit
    position.value = controllerMsg.position
    velocity.value = controllerMsg.velocity
    current.value = controllerMsg.current
  }
})

watch(scienceMessage, (msg) => {
  if (msg && (msg as ControllerStateMessage).type == props.msgType) {
    const controllerMsg = msg as ControllerStateMessage;
    name.value = controllerMsg.name
    state.value = controllerMsg.state
    error.value = controllerMsg.error
    limits.value = controllerMsg.limit_hit
    position.value = controllerMsg.position
    velocity.value = controllerMsg.velocity
    current.value = controllerMsg.current
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

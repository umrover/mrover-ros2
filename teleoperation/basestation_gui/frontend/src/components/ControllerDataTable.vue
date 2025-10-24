<template>
  <div class='wrap border border-2 p-2 rounded'>
    <h3 class="m-0 p-0 mb-1">{{ header }}</h3>
    <table class='table table-bordered m-0 p-0 border' style='table-layout: fixed; width: auto'>
      <tbody>
      <tr>
        <th class='table-secondary'>Motor</th>
        <td v-for='(name, i) in name' :key='i' class="m-0 px-1">
          {{ name }}
        </td>
      </tr>
      <tr>
        <th class='table-secondary'>State</th>
        <td v-for='(state, i) in state' :key='i'>
          {{ state }}
        </td>
      </tr>
      <tr>
        <th class='table-secondary'>Error</th>
        <td v-for='(error, i) in error' :key='i'>
          {{ error }}
        </td>
      </tr>
      <tr>
        <th class='table-secondary'>Limit Hit</th>
        <td v-for='(limits, i) in limits' :key='i'>
          {{ limits }}
        </td>
      </tr>
      </tbody>
    </table>
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

const armMessage = computed(() => messages.value['arm'])
const driveMessage = computed(() => messages.value['drive'])

watch(armMessage, (msg) => {
  if (msg && (msg as ControllerStateMessage).type == props.msgType) {
    const controllerMsg = msg as ControllerStateMessage;
    name.value = controllerMsg.name
    state.value = controllerMsg.state
    error.value = controllerMsg.error
    limits.value = controllerMsg.limit_hit
  }
})

watch(driveMessage, (msg) => {
  if (msg && (msg as ControllerStateMessage).type == props.msgType) {
    const controllerMsg = msg as ControllerStateMessage;
    name.value = controllerMsg.name
    state.value = controllerMsg.state
    error.value = controllerMsg.error
    limits.value = controllerMsg.limit_hit
  }
})
</script>

<style scoped>
.wrap {
  display: inline-block;
  align-content: center;
  min-width: 350px;
}
</style>

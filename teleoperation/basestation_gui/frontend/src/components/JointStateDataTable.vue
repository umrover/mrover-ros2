<template>
    <div class='wrap'>
      <div>
        <h3>{{ header }}</h3>
      </div>
      <table class='table table-bordered' style='table-layout: fixed; width: auto'>
        <tbody>
        <tr>
          <th class='table-secondary'>Motor</th>
          <td v-for='(name, i) in name' :key='i'>
            {{ name }}
          </td>
        </tr>
        <tr>
          <th class='table-secondary'>Pos.</th>
          <td v-for='(pos, i) in position' :key='i'>
            {{ pos }}
          </td>
        </tr>
        <tr>
          <th class='table-secondary'>Vel.</th>
          <td v-for='(vel, i) in velocity' :key='i'>
            {{ vel }}
          </td>
        </tr>
        <tr>
          <th class='table-secondary'>Effort</th>
          <td v-for='(eff, i) in effort' :key='i'>
            {{ eff }}
          </td>
        </tr>
        </tbody>
      </table>
    </div>
  </template>
  
  <script lang='ts' setup>
  import { defineComponent, ref, computed, watch } from 'vue'
  import { useWebsocketStore } from '@/stores/websocket'
  import { storeToRefs } from 'pinia'
  
  const props = defineProps({
    header: {
      type: String,
      required: true,
    },
    msgType: {
      type: String,
      required: true,
    },
    websocketId: {
      type: String,
      required: true,
    }
  })

  const websocketStore = useWebsocketStore()
  const { messages } = storeToRefs(websocketStore)

  const name = ref<string[]>([])
  const position = ref<number[]>([])
  const velocity = ref<number[]>([])
  const effort = ref<number[]>([])

  const message = computed(() => messages.value[props.websocketId])

  watch(message, (msg) => {
    if (msg && msg.type == props.msgType) {
      name.value = msg.name
      position.value = msg.position
      velocity.value = msg.velocity
      effort.value = msg.effort
    }
  })
  </script>
  
  <style scoped>
  .wrap {
    display: inline-block;
    align-content: center;
    margin: 0px 10px 0px 10px;
  }
  </style>
<template>
  <div class="wrapper view-wrapper">
    <div class="island p-2 rounded sensors">
      <SensorData :site="site" />
    </div>
  </div>
</template>

<script lang="ts" setup>
import SensorData from '../components/SensorData.vue'
import { ref, onMounted, onUnmounted } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'

const site = ref(0 as number)

const topics = ['science']
const websocketStore = useWebsocketStore()

onMounted(() => {
  window.setTimeout(() => {
    for (const topic of topics)
      websocketStore.setupWebSocket(topic)
  }, 0)
})

onUnmounted(() => {
  websocketStore.closeWebSocket('science')
})
</script>

<style scoped>
.wrapper {
  display: flex;
  flex-direction: column;
  gap: 10px;
  width: 100%;
  height: 100%;
}

.sensors {
  flex-grow: 1;
}
</style>
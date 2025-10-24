<template>
  <div class="view-wrapper">
    <ArmControls class="island p-2"/>
    <!-- <Rover3D /> -->
  </div>
</template>

<script lang="ts" setup>
import ArmControls from '../components/ArmControls.vue';
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const waypointsMessage = computed(() => messages.value['waypoints'])

onMounted(() => {
  websocketStore.setupWebSocket('arm')
  websocketStore.setupWebSocket('nav')

  setTimeout(() => {
    websocketStore.closeWebSocket('nav')
    spamTestMessages()
  }, 1000)
})

onUnmounted(() => {
  websocketStore.closeWebSocket('arm')
  websocketStore.closeWebSocket('nav')
})

watch(waypointsMessage, (msg) => {
  console.log(msg)
})

const spamTestMessages = () => {
  const interval = setInterval(() => {
    websocketStore.sendMessage('waypoints', {
      type: 'debug',
      timestamp: new Date().toISOString(),
    })
  }, 1000)
  setTimeout(() => clearInterval(interval), 5000)
}
</script>

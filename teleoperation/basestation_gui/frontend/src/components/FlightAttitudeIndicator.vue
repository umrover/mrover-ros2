<template>
  <div class="d-flex justify-content-center">
    <Attitude :size="200" :pitch="pitch" :roll="roll" />
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch } from 'vue'
import { Attitude } from 'vue-flight-indicators'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const pitch = ref(0)
const roll = ref(0)

const navMessage = computed(() => messages.value['nav'])

watch(navMessage, (msg) => {
  if (msg && msg.type == 'orientation') {
    const [qx, qy, qz, qw] = msg.orientation
    pitch.value = (Math.asin(2 * (qx * qz - qy * qw)) * 180) / Math.PI
    roll.value =
      (Math.atan2(2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)) *
        180) /
      Math.PI
  }
})
</script>

<style scoped></style>

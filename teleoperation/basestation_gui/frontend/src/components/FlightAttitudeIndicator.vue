<template>
  <div ref="wrapperRef" class="d-flex justify-content-center align-items-center w-100 h-100">
    <Attitude :size="indicatorSize" :pitch="pitch" :roll="roll" />
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch } from 'vue'
import { Attitude } from 'vue-flight-indicators'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import type { OrientationMessage } from '@/types/websocket'

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const pitch = ref(0)
const roll = ref(0)
const indicatorSize = ref(Math.min(window.innerHeight * 0.15, 180))

const navMessage = computed(() => messages.value['nav'])

watch(navMessage, (msg) => {
  if (msg && (msg as OrientationMessage).type == 'orientation') {
    const orientationMsg = msg as OrientationMessage;
    const [qx, qy, qz, qw] = orientationMsg.orientation
    pitch.value = (Math.asin(2 * (qx * qz - qy * qw)) * 180) / Math.PI
    roll.value =
      (Math.atan2(2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)) *
        180) /
      Math.PI
  }
})
</script>

<style scoped></style>

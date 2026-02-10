<template>
  <div class="d-flex justify-content-center" style="user-select: none;">
    <div
      v-if="Object.keys(connectionStatus as Record<string, any>).length > 0"
      class="justify-content-center align-items-center border border-2 rounded px-1 me-1"
    >
      <div class="d-flex align-items-center gap-2">
        <IndicatorDot :is-active="true" />
        <span class="ws-label">= TX</span>
      </div>
      <div class="d-flex align-items-center gap-2">
        <IndicatorDot :is-active="false" />
        <span class="ws-label">= RX</span>
      </div>
    </div>
    <div class="gap-1 d-flex">
      <div
        v-for="(status, id) in connectionStatus"
        :key="id"
        :class="[
          'mx-0 flex-column align-items-center border border-2 rounded p-1',
          status === 'disconnected' ? 'bg-warning' : ''
        ]"
      >
        <p class="ws-connection-name m-0 p-0 text-center">{{ getAlias(id) }}</p>

        <div class="d-flex justify-content-center align-items-center gap-2">
          <div
            class="rounded-circle"
            :class="isFlashingOut(id) ? 'bg-success' : 'bg-secondary'"
            style="width: 16px; height: 16px"
          ></div>
          <div
            class="rounded-circle"
            :class="isFlashingIn(id) ? 'bg-danger' : 'bg-secondary'"
            style="width: 16px; height: 16px"
          ></div>
        </div>
      </div>
    </div>
    <div class="border border-2 rounded px-1 ms-1 d-flex flex-column justify-content-center">
      <div class="d-flex gap-2"><span class="text-success fw-semibold">TX</span> {{ txMsgRate }}/s {{ formatBytes(txByteRate) }}/s</div>
      <div class="d-flex gap-2"><span class="text-danger fw-semibold">RX</span> {{ rxMsgRate }}/s {{ formatBytes(rxByteRate) }}/s</div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, onMounted, onBeforeUnmount } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import IndicatorDot from './IndicatorDot.vue'

const websocketStore = useWebsocketStore()
const { connectionStatus, incomingMessages, outgoingMessages, incomingBytes, outgoingBytes } = storeToRefs(websocketStore)
const { isFlashingIn, isFlashingOut } = websocketStore

const aliasMap: Record<string, string> = {
  arm: 'arm',
  drive: 'drv',
  chassis: 'cha',
  nav: 'nav',
  science: 'sci',
  latency: 'lat',
  recording: 'rec',
}

const getAlias = (id: string): string => aliasMap[id] || id

const txMsgRate = ref(0)
const rxMsgRate = ref(0)
const txByteRate = ref(0)
const rxByteRate = ref(0)

const INTERVAL_MS = 500
const RATE_MULTIPLIER = 1000 / INTERVAL_MS

let prevInMsgs = 0
let prevOutMsgs = 0
let prevInBytes = 0
let prevOutBytes = 0
let interval: number | undefined

function sumValues(obj: Record<string, number>): number {
  return Object.values(obj).reduce((a, b) => a + b, 0)
}

function updateRates() {
  const inMsgs = sumValues(incomingMessages.value)
  const outMsgs = sumValues(outgoingMessages.value)
  const inBytes = sumValues(incomingBytes.value)
  const outBytes = sumValues(outgoingBytes.value)

  rxMsgRate.value = (inMsgs - prevInMsgs) * RATE_MULTIPLIER
  txMsgRate.value = (outMsgs - prevOutMsgs) * RATE_MULTIPLIER
  rxByteRate.value = (inBytes - prevInBytes) * RATE_MULTIPLIER
  txByteRate.value = (outBytes - prevOutBytes) * RATE_MULTIPLIER

  prevInMsgs = inMsgs
  prevOutMsgs = outMsgs
  prevInBytes = inBytes
  prevOutBytes = outBytes
}

function formatBytes(bytes: number): string {
  if (bytes < 1024) return `${bytes}B`
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)}KB`
  return `${(bytes / (1024 * 1024)).toFixed(1)}MB`
}

onMounted(() => {
  interval = window.setInterval(updateRates, INTERVAL_MS)
})

onBeforeUnmount(() => {
  if (interval) window.clearInterval(interval)
})
</script>

<style scoped>
.ws-label {
  font-weight: 600;
}

.ws-connection-name {
  font-weight: 700;
}
</style>

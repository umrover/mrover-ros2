<template>
  <div class="flex justify-center" style="user-select: none;">
    <div
      v-if="Object.keys(connectionStatus as Record<string, any>).length > 0"
      class="justify-center items-center border border-2 rounded px-1 mr-1"
    >
      <div class="flex items-center gap-2">
        <IndicatorDot :is-active="true" />
        <span class="font-semibold">= TX</span>
      </div>
      <div class="flex items-center gap-2">
        <IndicatorDot :is-active="false" />
        <span class="font-semibold">= RX</span>
      </div>
    </div>
    <div class="gap-1 flex">
      <div
        v-for="(status, id) in connectionStatus"
        :key="id"
        :class="[
          'mx-0 flex-col items-center border border-2 rounded p-1',
          status === 'disconnected' ? 'bg-cmd-warning' : ''
        ]"
      >
        <p class="font-bold text-center">{{ getAlias(id) }}</p>

        <div class="flex justify-center items-center gap-2">
          <div
            class="rounded-full"
            :class="flashOutDisplay[id] ? 'bg-cmd-success' : 'bg-cmd-secondary'"
            style="width: 16px; height: 16px"
          ></div>
          <div
            class="rounded-full"
            :class="flashInDisplay[id] ? 'bg-cmd-danger' : 'bg-cmd-secondary'"
            style="width: 16px; height: 16px"
          ></div>
        </div>
      </div>
    </div>
    <div class="border border-2 rounded px-1 ml-1 flex flex-col justify-center">
      <div class="flex gap-2"><span class="text-cmd-success font-semibold">TX</span> {{ txMsgRate }}/s {{ formatBytes(txByteRate) }}/s</div>
      <div class="flex gap-2"><span class="text-cmd-danger font-semibold">RX</span> {{ rxMsgRate }}/s {{ formatBytes(rxByteRate) }}/s</div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, onMounted, onBeforeUnmount } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import IndicatorDot from './IndicatorDot.vue'

const websocketStore = useWebsocketStore()
const { connectionStatus } = storeToRefs(websocketStore)
const { getFlashIn, getFlashOut, getIncomingMessages, getOutgoingMessages, getIncomingBytes, getOutgoingBytes } = websocketStore

const flashInDisplay = ref<Record<string, boolean>>({})
const flashOutDisplay = ref<Record<string, boolean>>({})

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
  const inMsgs = sumValues(getIncomingMessages())
  const outMsgs = sumValues(getOutgoingMessages())
  const inBytes = sumValues(getIncomingBytes())
  const outBytes = sumValues(getOutgoingBytes())

  rxMsgRate.value = (inMsgs - prevInMsgs) * RATE_MULTIPLIER
  txMsgRate.value = (outMsgs - prevOutMsgs) * RATE_MULTIPLIER
  rxByteRate.value = (inBytes - prevInBytes) * RATE_MULTIPLIER
  txByteRate.value = (outBytes - prevOutBytes) * RATE_MULTIPLIER

  prevInMsgs = inMsgs
  prevOutMsgs = outMsgs
  prevInBytes = inBytes
  prevOutBytes = outBytes

  for (const id of Object.keys(connectionStatus.value)) {
    flashInDisplay.value[id] = getFlashIn(id)
    flashOutDisplay.value[id] = getFlashOut(id)
  }
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


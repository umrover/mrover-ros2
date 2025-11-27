<template>
  <div class="d-flex flex-column gap-2 p-3">
    <h4 class="m-0">Latency Benchmark</h4>

    <div class="d-flex gap-2 align-items-center flex-wrap">
      <button
        class="btn btn-sm"
        :class="isRunning ? 'btn-danger' : 'btn-success'"
        @click="toggleBenchmark"
      >
        {{ isRunning ? 'Stop' : 'Start' }} Benchmark
      </button>
      <button
        class="btn btn-sm btn-secondary"
        @click="resetStats"
      >
        Reset
      </button>
      <div class="d-flex align-items-center gap-1">
        <input
          v-model.number="frequency"
          type="number"
          class="form-control form-control-sm"
          style="width: 70px"
          min="1"
          max="1000"
        />
        <span class="text-nowrap">Hz</span>
      </div>
    </div>

    <div class="d-flex flex-column gap-1">
      <h6 class="mb-1 mt-2">Connection</h6>
      <div class="d-flex justify-content-between">
        <span>Status:</span>
        <span :class="connectionStatus === 'connected' ? 'text-success' : 'text-danger'">
          {{ connectionStatus }}
        </span>
      </div>

      <h6 class="mb-1 mt-2">Latency</h6>
      <div class="d-flex justify-content-between">
        <span>Current RTT:</span>
        <span class="font-monospace">{{ currentLatency }} ms</span>
      </div>
      <div class="d-flex justify-content-between">
        <span>Average RTT:</span>
        <span class="font-monospace">{{ averageLatency }} ms</span>
      </div>
      <div class="d-flex justify-content-between">
        <span>Min RTT:</span>
        <span class="font-monospace">{{ minLatency }} ms</span>
      </div>
      <div class="d-flex justify-content-between">
        <span>Max RTT:</span>
        <span class="font-monospace">{{ maxLatency }} ms</span>
      </div>
      <div class="d-flex justify-content-between">
        <span>Std Dev:</span>
        <span class="font-monospace">{{ stdDevLatency }} ms</span>
      </div>
      <div class="d-flex justify-content-between">
        <span>P95 RTT:</span>
        <span class="font-monospace">{{ p95Latency }} ms</span>
      </div>
      <div class="d-flex justify-content-between">
        <span>P99 RTT:</span>
        <span class="font-monospace">{{ p99Latency }} ms</span>
      </div>

      <h6 class="mb-1 mt-2">Throughput</h6>
      <div class="d-flex justify-content-between">
        <span>Target Frequency:</span>
        <span class="font-monospace">{{ frequency }} Hz</span>
      </div>
      <div class="d-flex justify-content-between">
        <span>Actual Frequency:</span>
        <span class="font-monospace" :class="actualFrequencyColor">{{ actualFrequency }} Hz</span>
      </div>
      <div class="d-flex justify-content-between">
        <span>Messages/sec:</span>
        <span class="font-monospace">{{ messagesPerSecond }}</span>
      </div>
      <div class="d-flex justify-content-between">
        <span>Upload:</span>
        <span class="font-monospace">{{ uploadThroughput }} KB/s</span>
      </div>
      <div class="d-flex justify-content-between">
        <span>Download:</span>
        <span class="font-monospace">{{ downloadThroughput }} KB/s</span>
      </div>
      <div class="d-flex justify-content-between">
        <span>Total Sent:</span>
        <span class="font-monospace">{{ totalBytesSent }} KB</span>
      </div>
      <div class="d-flex justify-content-between">
        <span>Total Received:</span>
        <span class="font-monospace">{{ totalBytesReceived }} KB</span>
      </div>

      <h6 class="mb-1 mt-2">Reliability</h6>
      <div class="d-flex justify-content-between">
        <span>Packets Sent:</span>
        <span class="font-monospace">{{ packetsSent }}</span>
      </div>
      <div class="d-flex justify-content-between">
        <span>Packets Received:</span>
        <span class="font-monospace">{{ packetsReceived }}</span>
      </div>
      <div class="d-flex justify-content-between">
        <span>Packets Lost:</span>
        <span class="font-monospace" :class="packetsLost > 0 ? 'text-danger' : 'text-success'">
          {{ packetsLost }}
        </span>
      </div>
      <div class="d-flex justify-content-between">
        <span>Packet Loss:</span>
        <span class="font-monospace" :class="parseFloat(packetLossRate) > 0 ? 'text-warning' : 'text-success'">
          {{ packetLossRate }}%
        </span>
      </div>
      <div class="d-flex justify-content-between">
        <span>Out of Order:</span>
        <span class="font-monospace">{{ outOfOrderCount }}</span>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, onUnmounted, watch } from 'vue'
import { encode, decode } from '@msgpack/msgpack'
import { useWebsocketStore } from '@/stores/websocket'

const websocketStore = useWebsocketStore()
let ws: WebSocket | null = null

const isRunning = ref(false)
const currentLatency = ref(0)
const latencies = ref<number[]>([])
const sampleCount = ref(0)
const frequency = ref(10)

const bytesSentTotal = ref(0)
const bytesReceivedTotal = ref(0)
const messagesSent = ref(0)
const messagesReceived = ref(0)

const bytesWindow = ref<Array<{ timestamp: number; sent: number; received: number; sequence: number }>>([])
const windowDuration = 1000

const sequenceNumber = ref(0)
const packetsSent = ref(0)
const packetsReceived = ref(0)
const packetsLost = ref(0)
const outOfOrderCount = ref(0)
const lastReceivedSequence = ref(-1)
const pendingPackets = ref(new Map<number, number>())
const receivedPackets = ref(new Set<number>())

const sendTimestamps = ref<number[]>([])
const actualFrequency = ref(0)

let pingInterval: number | undefined
let timeoutCheckInterval: number | undefined
let usingAnimationFrame = false

const PACKET_TIMEOUT_MS = 500

const connectionStatus = ref('disconnected')

let flashInTimer: number | undefined
let flashOutTimer: number | undefined

// Helpers for store updates to keep UI in sync
const updateStoreStatus = (status: string) => {
  websocketStore.setConnectionStatus('latency', status)
}

const updateStoreActivityIn = () => {
  websocketStore.setLastIncomingActivity('latency', Date.now())
  websocketStore.setFlashIn('latency', true)
  
  if (flashInTimer !== undefined) {
    clearTimeout(flashInTimer)
  }
  flashInTimer = setTimeout(() => {
    websocketStore.clearFlashIn('latency')
    flashInTimer = undefined
  }, 200)
}

const updateStoreActivityOut = () => {
  websocketStore.setLastOutgoingActivity('latency', Date.now())
  websocketStore.setFlashOut('latency', true)

  if (flashOutTimer !== undefined) {
    clearTimeout(flashOutTimer)
  }
  flashOutTimer = setTimeout(() => {
    websocketStore.clearFlashOut('latency')
    flashOutTimer = undefined
  }, 200)
}

interface PongMessage {
  type: string
  sequence?: number
  timestamp: number
}

const connectWebSocket = () => {
  ws = new WebSocket('ws://localhost:8000/latency')
  ws.binaryType = 'arraybuffer'

  ws.onopen = () => {
    console.log('WebSocket connected')
    connectionStatus.value = 'connected'
    updateStoreStatus('connected')
  }

  ws.onmessage = (event) => {
    updateStoreActivityIn()
    const data = decode(new Uint8Array(event.data)) as PongMessage

    if (data.type === 'pong') {
      const seq = data.sequence
      if (seq !== undefined) {
        if (receivedPackets.value.has(seq)) {
          return
        }

        receivedPackets.value.add(seq)

        if (pendingPackets.value.has(seq)) {
          pendingPackets.value.delete(seq)
          packetsReceived.value++
        }

        if (seq < lastReceivedSequence.value) {
          outOfOrderCount.value++
        }
        lastReceivedSequence.value = Math.max(lastReceivedSequence.value, seq)
      }

      const rtt = performance.now() - data.timestamp

      currentLatency.value = parseFloat(rtt.toFixed(2))
      latencies.value.push(rtt)
      sampleCount.value++

      const receivedBytes = new Uint8Array(event.data).length
      bytesReceivedTotal.value += receivedBytes
      messagesReceived.value++

      const windowEntry = bytesWindow.value.find(entry => entry.sequence === seq)
      if (windowEntry) {
        windowEntry.received = receivedBytes
      }
    }
  }

  ws.onclose = () => {
    console.log('WebSocket disconnected')
    connectionStatus.value = 'disconnected'
    updateStoreStatus('disconnected')
  }

  ws.onerror = (error) => {
    console.error('WebSocket error:', error)
    connectionStatus.value = 'disconnected'
    updateStoreStatus('disconnected')
  }
}

const averageLatency = computed(() => {
  if (latencies.value.length === 0) return '0.00'
  const sum = latencies.value.reduce((a, b) => a + b, 0)
  return (sum / latencies.value.length).toFixed(2)
})

const minLatency = computed(() => {
  if (latencies.value.length === 0) return '0.00'
  return Math.min(...latencies.value).toFixed(2)
})

const maxLatency = computed(() => {
  if (latencies.value.length === 0) return '0.00'
  return Math.max(...latencies.value).toFixed(2)
})

const stdDevLatency = computed(() => {
  if (latencies.value.length === 0) return '0.00'
  const avg = latencies.value.reduce((a, b) => a + b, 0) / latencies.value.length
  const variance = latencies.value.reduce((sum, val) => sum + Math.pow(val - avg, 2), 0) / latencies.value.length
  return Math.sqrt(variance).toFixed(2)
})

const p95Latency = computed(() => {
  if (latencies.value.length === 0) return '0.00'
  const sorted = [...latencies.value].sort((a, b) => a - b)
  const index = Math.ceil(sorted.length * 0.95) - 1
  return sorted[index].toFixed(2)
})

const p99Latency = computed(() => {
  if (latencies.value.length === 0) return '0.00'
  const sorted = [...latencies.value].sort((a, b) => a - b)
  const index = Math.ceil(sorted.length * 0.99) - 1
  return sorted[index].toFixed(2)
})

const messagesPerSecond = computed(() => {
  const now = Date.now()
  const recentWindow = bytesWindow.value.filter(
    entry => now - entry.timestamp < windowDuration
  )
  return recentWindow.length.toFixed(1)
})

const uploadThroughput = computed(() => {
  const now = Date.now()
  const recentWindow = bytesWindow.value.filter(
    entry => now - entry.timestamp < windowDuration
  )
  const totalBytes = recentWindow.reduce((sum, entry) => sum + entry.sent, 0)
  const kbps = (totalBytes / 1024) / (windowDuration / 1000)
  return kbps.toFixed(2)
})

const downloadThroughput = computed(() => {
  const now = Date.now()
  const recentWindow = bytesWindow.value.filter(
    entry => now - entry.timestamp < windowDuration
  )
  const totalBytes = recentWindow.reduce((sum, entry) => sum + entry.received, 0)
  const kbps = (totalBytes / 1024) / (windowDuration / 1000)
  return kbps.toFixed(2)
})

const totalBytesSent = computed(() => (bytesSentTotal.value / 1024).toFixed(2))
const totalBytesReceived = computed(() => (bytesReceivedTotal.value / 1024).toFixed(2))

const packetLossRate = computed(() => {
  if (packetsSent.value === 0) return '0.00'
  const lossRate = (packetsLost.value / packetsSent.value) * 100
  return lossRate.toFixed(2)
})

const actualFrequencyColor = computed(() => {
  const diff = Math.abs(actualFrequency.value - frequency.value)
  const percentDiff = (diff / frequency.value) * 100
  if (percentDiff > 20) return 'text-danger'
  if (percentDiff > 10) return 'text-warning'
  return 'text-success'
})

watch(frequency, () => {
  if (isRunning.value) {
    latencies.value = []
    currentLatency.value = 0
    packetsSent.value = 0
    packetsReceived.value = 0
    packetsLost.value = 0
    outOfOrderCount.value = 0
    lastReceivedSequence.value = -1
    pendingPackets.value.clear()
    receivedPackets.value.clear()
    stopBenchmark()
    startBenchmark()
  }
})


const sendPing = () => {
  if (!ws || ws.readyState !== WebSocket.OPEN) {
    console.warn('WebSocket not connected')
    return
  }

  const now = Date.now()
  sendTimestamps.value.push(now)

  const recentSends = sendTimestamps.value.filter(ts => now - ts < 1000)
  sendTimestamps.value = recentSends
  actualFrequency.value = parseFloat(recentSends.length.toFixed(1))

  const seq = sequenceNumber.value++
  const timestamp = performance.now()

  const message = {
    type: 'ping',
    timestamp: timestamp,
    sequence: seq,
    payload: {
      controller_axes: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
      controller_buttons: [0, 1, 0, 1, 0, 1, 0, 1],
      position: { x: 1.234, y: 5.678, z: 9.012 },
      orientation: { x: 0.0, y: 0.0, z: 0.707, w: 0.707 },
      metadata: {
        frame_id: "base_link",
        sequence: sampleCount.value,
      },
      joint_states: Array(20).fill(null).map((_, i) => ({
        name: `joint_${i}`,
        position: Math.random(),
        velocity: Math.random(),
        effort: Math.random()
      })),
      extra_data: Array(50).fill(null).map((_, i) => ({
        id: i,
        value: Math.random() * 100,
        timestamp: Date.now()
      }))
    }
  }

  pendingPackets.value.set(seq, now)
  packetsSent.value++

  const packed = encode(message)
  const sentBytes = packed.length
  bytesSentTotal.value += sentBytes
  messagesSent.value++

  bytesWindow.value.push({
    timestamp: now,
    sent: sentBytes,
    received: 0,
    sequence: seq
  })

  const cutoffTime = now - windowDuration
  while (bytesWindow.value.length > 0 && bytesWindow.value[0].timestamp < cutoffTime) {
    bytesWindow.value.shift()
  }

  updateStoreActivityOut()
  ws.send(packed)
}

const toggleBenchmark = () => {
  if (isRunning.value) {
    stopBenchmark()
  } else {
    startBenchmark()
  }
}

const checkForLostPackets = () => {
  const now = Date.now()
  const timeoutThreshold = now - PACKET_TIMEOUT_MS

  for (const [seq, timestamp] of pendingPackets.value.entries()) {
    if (timestamp < timeoutThreshold) {
      pendingPackets.value.delete(seq)
      packetsLost.value++
    }
  }
}

const startBenchmark = () => {
  isRunning.value = true

  timeoutCheckInterval = window.setInterval(() => {
    checkForLostPackets()
  }, 1000)

  if (frequency.value <= 250) {
    usingAnimationFrame = false
    const intervalMs = 1000 / frequency.value
    pingInterval = window.setInterval(() => {
      sendPing()
    }, intervalMs)
  } else {
    usingAnimationFrame = true
    let lastTime = performance.now()
    const targetInterval = 1000 / frequency.value

    const accurateSend = () => {
      if (!isRunning.value) return

      const currentTime = performance.now()

      while (currentTime - lastTime >= targetInterval) {
        sendPing()
        lastTime += targetInterval
      }

      pingInterval = window.requestAnimationFrame(accurateSend)
    }

    pingInterval = window.requestAnimationFrame(accurateSend)
  }
}

const stopBenchmark = () => {
  isRunning.value = false
  if (pingInterval !== undefined) {
    if (usingAnimationFrame) {
      cancelAnimationFrame(pingInterval)
    } else {
      clearInterval(pingInterval)
    }
    pingInterval = undefined
  }
  if (timeoutCheckInterval !== undefined) {
    clearInterval(timeoutCheckInterval)
    timeoutCheckInterval = undefined
  }
}

const resetStats = () => {
  latencies.value = []
  currentLatency.value = 0
  sampleCount.value = 0
  bytesSentTotal.value = 0
  bytesReceivedTotal.value = 0
  messagesSent.value = 0
  messagesReceived.value = 0
  bytesWindow.value = []
  sequenceNumber.value = 0
  packetsSent.value = 0
  packetsReceived.value = 0
  packetsLost.value = 0
  outOfOrderCount.value = 0
  lastReceivedSequence.value = -1
  pendingPackets.value.clear()
  receivedPackets.value.clear()
  sendTimestamps.value = []
  actualFrequency.value = 0
}

connectWebSocket()

onUnmounted(() => {
  stopBenchmark()
  if (ws) {
    ws.close()
  }
})
</script>

<style scoped>
.font-monospace {
  font-family: 'Courier New', monospace;
}
</style>

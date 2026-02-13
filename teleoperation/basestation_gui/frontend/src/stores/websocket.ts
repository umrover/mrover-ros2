import { defineStore } from 'pinia'
import { ref, shallowRef } from 'vue'
import { encode, decode } from '@msgpack/msgpack'

const webSockets: Record<string, WebSocket> = {}
const refCounts: Record<string, number> = {}
const flashTimersIn: Record<string, ReturnType<typeof setTimeout>> = {}
const flashTimersOut: Record<string, ReturnType<typeof setTimeout>> = {}
const reconnectTimers: Record<string, ReturnType<typeof setTimeout>> = {}
const reconnectAttempts: Record<string, number> = {}
const closedIntentionally: Set<string> = new Set()

const _lastIncomingActivity: Record<string, number> = {}
const _lastOutgoingActivity: Record<string, number> = {}
const _flashIn: Record<string, boolean> = {}
const _flashOut: Record<string, boolean> = {}
const _incomingMessages: Record<string, number> = {}
const _outgoingMessages: Record<string, number> = {}
const _incomingBytes: Record<string, number> = {}
const _outgoingBytes: Record<string, number> = {}

const MAX_RECONNECT_ATTEMPTS = 10
const BASE_RECONNECT_DELAY_MS = 1000
const MAX_RECONNECT_DELAY_MS = 30000

interface WebsocketStoreActions {
  setMessage: (id: string, message: unknown) => void
  setConnectionStatus: (id: string, status: string) => void
  setLastIncomingActivity: (id: string, timestamp: number) => void
  setLastOutgoingActivity: (id: string, timestamp: number) => void
  setFlashIn: (id: string, value: boolean) => void
  setFlashOut: (id: string, value: boolean) => void
  clearFlashIn: (id: string) => void
  clearFlashOut: (id: string) => void
  addIncomingMetrics: (id: string, bytes: number) => void
  addOutgoingMetrics: (id: string, bytes: number) => void
}

function debounceFlashClear(id: string, type: 'in' | 'out', store: WebsocketStoreActions) {
  const timers = type === 'in' ? flashTimersIn : flashTimersOut
  if (timers[id]) {
    clearTimeout(timers[id])
  }
  timers[id] = setTimeout(() => {
    if (type === 'in') {
      store.clearFlashIn(id)
    } else {
      store.clearFlashOut(id)
    }
    delete timers[id]
  }, 200)
}

function setupWebsocket(id: string, store: WebsocketStoreActions) {
  if (!id) {
    console.error('Invalid WebSocket ID passed:', id)
    return
  }

  if (closedIntentionally.has(id)) {
    return
  }

  if (webSockets[id]) {
    console.warn(`WebSocket with ID ${id} already exists.`)
    return
  }

  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
  const socket = new WebSocket(`${protocol}//${window.location.host}/ws/${id}`)
  socket.binaryType = 'arraybuffer'

  socket.onopen = () => {
    console.log(`WebSocket ${id} Connected`)
    store.setConnectionStatus(id, 'connected')
    reconnectAttempts[id] = 0
  }

  socket.onmessage = event => {
    const data = new Uint8Array(event.data)
    const message = decode(data)
    store.setMessage(id, message)
    store.setLastIncomingActivity(id, Date.now())
    store.setFlashIn(id, true)
    store.addIncomingMetrics(id, data.byteLength)
    debounceFlashClear(id, 'in', store)
  }

  socket.onclose = e => {
    store.setConnectionStatus(id, 'disconnected')
    delete webSockets[id]

    if (closedIntentionally.has(id)) {
      closedIntentionally.delete(id)
      return
    }

    const attempts = (reconnectAttempts[id] || 0) + 1
    reconnectAttempts[id] = attempts

    if (attempts > MAX_RECONNECT_ATTEMPTS) {
      console.error(`WebSocket ${id} max reconnect attempts reached, giving up`)
      store.setConnectionStatus(id, 'failed')
      return
    }

    const delay = Math.min(BASE_RECONNECT_DELAY_MS * Math.pow(2, attempts - 1), MAX_RECONNECT_DELAY_MS)
    console.log(`WebSocket ${id} closed. Reconnecting in ${delay}ms (attempt ${attempts}/${MAX_RECONNECT_ATTEMPTS})...`, e.reason)

    reconnectTimers[id] = setTimeout(() => {
      delete reconnectTimers[id]
      setupWebsocket(id, store)
    }, delay)
  }

  socket.onerror = error => {
    console.error(`WebSocket ${id} encountered error`, error)
    store.setConnectionStatus(id, 'disconnected')
    socket.close()
  }

  const originalSend = socket.send
  socket.send = function (data: string | ArrayBufferLike | Blob | ArrayBufferView) {
    if (this.readyState !== WebSocket.OPEN) {
      console.warn(`WebSocket [${id}] is not open. Current state: ${this.readyState}`)
      return
    }
    store.setLastOutgoingActivity(id, Date.now())
    store.setFlashOut(id, true)
    const byteLength = data instanceof Blob ? data.size : (data as ArrayBufferView).byteLength ?? (data as ArrayBuffer).byteLength ?? 0
    store.addOutgoingMetrics(id, byteLength)
    debounceFlashClear(id, 'out', store)
    return originalSend.call(this, data)
  }

  webSockets[id] = socket
}

export const useWebsocketStore = defineStore('websocket', () => {
  const messages = shallowRef<Record<string, unknown>>({})
  const connectionStatus = ref<Record<string, string>>({})

  function setMessage(id: string, message: unknown) {
    messages.value = { ...messages.value, [id]: message }
  }

  function setConnectionStatus(id: string, status: string) {
    connectionStatus.value = { ...connectionStatus.value, [id]: status }
  }

  function setFlashIn(id: string, value: boolean) {
    _flashIn[id] = value
  }

  function setFlashOut(id: string, value: boolean) {
    _flashOut[id] = value
  }

  function clearFlashIn(id: string) {
    _flashIn[id] = false
  }

  function clearFlashOut(id: string) {
    _flashOut[id] = false
  }

  function setLastIncomingActivity(id: string, timestamp: number) {
    _lastIncomingActivity[id] = timestamp
  }

  function setLastOutgoingActivity(id: string, timestamp: number) {
    _lastOutgoingActivity[id] = timestamp
  }

  function addIncomingMetrics(id: string, bytes: number) {
    _incomingMessages[id] = (_incomingMessages[id] || 0) + 1
    _incomingBytes[id] = (_incomingBytes[id] || 0) + bytes
  }

  function addOutgoingMetrics(id: string, bytes: number) {
    _outgoingMessages[id] = (_outgoingMessages[id] || 0) + 1
    _outgoingBytes[id] = (_outgoingBytes[id] || 0) + bytes
  }

  function resetMetrics() {
    for (const k of Object.keys(_incomingMessages)) delete _incomingMessages[k]
    for (const k of Object.keys(_outgoingMessages)) delete _outgoingMessages[k]
    for (const k of Object.keys(_incomingBytes)) delete _incomingBytes[k]
    for (const k of Object.keys(_outgoingBytes)) delete _outgoingBytes[k]
  }

  function getFlashIn(id: string): boolean {
    return _flashIn[id] || false
  }

  function getFlashOut(id: string): boolean {
    return _flashOut[id] || false
  }

  function getIncomingMessages(): Record<string, number> {
    return { ..._incomingMessages }
  }

  function getOutgoingMessages(): Record<string, number> {
    return { ..._outgoingMessages }
  }

  function getIncomingBytes(): Record<string, number> {
    return { ..._incomingBytes }
  }

  function getOutgoingBytes(): Record<string, number> {
    return { ..._outgoingBytes }
  }

  function sendMessage(id: string, message: unknown) {
    const socket = webSockets[id]
    if (!socket) {
      console.error('WebSocket not found for id:', id)
      return
    }
    if (socket.readyState !== WebSocket.OPEN) {
      console.error(`WebSocket ${id} not ready (state: ${socket.readyState})`)
      return
    }
    const packed = encode(message)
    socket.send(packed)
  }

  function setupWebSocket(id: string) {
    refCounts[id] = (refCounts[id] || 0) + 1
    if (refCounts[id] > 1) {
      return
    }
    closedIntentionally.delete(id)
    reconnectAttempts[id] = 0
    setupWebsocket(id, {
      setMessage,
      setConnectionStatus,
      setLastIncomingActivity,
      setLastOutgoingActivity,
      setFlashIn,
      setFlashOut,
      clearFlashIn,
      clearFlashOut,
      addIncomingMetrics,
      addOutgoingMetrics
    })
  }

  function closeWebSocket(id: string) {
    if (refCounts[id]) {
      refCounts[id]--
      if (refCounts[id] > 0) {
        return
      }
      delete refCounts[id]
    }
    closedIntentionally.add(id)
    if (reconnectTimers[id]) {
      clearTimeout(reconnectTimers[id])
      delete reconnectTimers[id]
    }
    delete reconnectAttempts[id]
    if (webSockets[id]) {
      webSockets[id].close()
      delete webSockets[id]
    }
  }

  return {
    messages,
    connectionStatus,
    setMessage,
    setConnectionStatus,
    setFlashIn,
    setFlashOut,
    clearFlashIn,
    clearFlashOut,
    setLastIncomingActivity,
    setLastOutgoingActivity,
    addIncomingMetrics,
    addOutgoingMetrics,
    resetMetrics,
    getFlashIn,
    getFlashOut,
    getIncomingMessages,
    getOutgoingMessages,
    getIncomingBytes,
    getOutgoingBytes,
    sendMessage,
    setupWebSocket,
    closeWebSocket
  }
})

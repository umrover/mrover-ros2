import { defineStore } from 'pinia'
import { ref, computed } from 'vue'

const webSockets: Record<string, WebSocket> = {}
const flashTimersIn: Record<string, NodeJS.Timeout> = {}
const flashTimersOut: Record<string, NodeJS.Timeout> = {}

function debounceFlashClear(id: string, type: 'in' | 'out', store: any) {
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
  }, 100)
}

function setupWebsocket(id: string, store: any) {
  if (!id) {
    console.error('Invalid WebSocket ID passed:', id)
    return
  }

  if (webSockets[id]) {
    console.warn(`WebSocket with ID ${id} already exists.`)
    return
  }

  const socket = new WebSocket(`ws://localhost:8000/ws/${id}`)

  socket.onopen = () => {
    console.log(`WebSocket ${id} Connected`)
    store.setConnectionStatus(id, 'connected')
  }

  socket.onmessage = event => {
    const message = JSON.parse(event.data)
    store.setMessage(id, message)
    store.setLastIncomingActivity(id, Date.now())
    store.setFlashIn(id, true)
    debounceFlashClear(id, 'in', store)
  }

  socket.onclose = e => {
    console.log(
      `WebSocket ${id} closed. Reconnecting in 2 seconds...`,
      e.reason,
    )
    store.setConnectionStatus(id, 'disconnected')
    delete webSockets[id]
    setTimeout(() => {
      setupWebsocket(id, store)
    }, 2000)
  }

  socket.onerror = error => {
    console.error(`WebSocket ${id} encountered error`, error)
    store.setConnectionStatus(id, 'disconnected')
    socket.close()
  }

  // Wrap send to track outgoing data activity
  const originalSend = socket.send
  socket.send = function (data: string | ArrayBufferLike | Blob | ArrayBufferView) {
    if (this.readyState !== WebSocket.OPEN) {
      console.warn(`WebSocket [${id}] is not open. Current state: ${this.readyState}`)
      return
    }
    store.setLastOutgoingActivity(id, Date.now())
    store.setFlashOut(id, true)
    debounceFlashClear(id, 'out', store)
    return originalSend.call(this, data)
  }

  webSockets[id] = socket
}

export const useWebsocketStore = defineStore('websocket', () => {
  // State
  const messages = ref<Record<string, any>>({})
  const connectionStatus = ref<Record<string, string>>({})
  const lastIncomingActivity = ref<Record<string, number>>({})
  const lastOutgoingActivity = ref<Record<string, number>>({})
  const flashIn = ref<Record<string, boolean>>({})
  const flashOut = ref<Record<string, boolean>>({})

  // Getters
  const isFlashingIn = computed(() => (id: string) => flashIn.value[id] || false)
  const isFlashingOut = computed(() => (id: string) => flashOut.value[id] || false)

  // Actions
  function setMessage(id: string, message: any) {
    messages.value[id] = message
  }

  function setConnectionStatus(id: string, status: string) {
    connectionStatus.value[id] = status
  }

  function setFlashIn(id: string, value: boolean) {
    flashIn.value[id] = value
  }

  function setFlashOut(id: string, value: boolean) {
    flashOut.value[id] = value
  }

  function clearFlashIn(id: string) {
    flashIn.value[id] = false
  }

  function clearFlashOut(id: string) {
    flashOut.value[id] = false
  }

  function setLastIncomingActivity(id: string, timestamp: number) {
    lastIncomingActivity.value[id] = timestamp
  }

  function setLastOutgoingActivity(id: string, timestamp: number) {
    lastOutgoingActivity.value[id] = timestamp
  }

  function sendMessage(id: string, message: any) {
    const socket = webSockets[id]
    if (!socket) {
      console.log('websocket selection failed with id', id)
      return
    }
    if (socket.readyState === socket.CLOSED) {
      console.log('websocket ' + id + ' not ready')
      return
    }
    socket.send(JSON.stringify(message))
  }

  function setupWebSocket(id: string) {
    setupWebsocket(id, {
      setMessage,
      setConnectionStatus,
      setLastIncomingActivity,
      setLastOutgoingActivity,
      setFlashIn,
      setFlashOut,
      clearFlashIn,
      clearFlashOut
    })
  }

  function closeWebSocket(id: string) {
    if (webSockets[id]) {
      webSockets[id].close()
      delete webSockets[id]
    }
  }

  return {
    // State
    messages,
    connectionStatus,
    lastIncomingActivity,
    lastOutgoingActivity,
    flashIn,
    flashOut,
    // Getters
    isFlashingIn,
    isFlashingOut,
    // Actions
    setMessage,
    setConnectionStatus,
    setFlashIn,
    setFlashOut,
    clearFlashIn,
    clearFlashOut,
    setLastIncomingActivity,
    setLastOutgoingActivity,
    sendMessage,
    setupWebSocket,
    closeWebSocket
  }
})

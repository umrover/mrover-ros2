const webSockets = {} // Store WebSocket instances dynamically
const flashTimersIn = {}
const flashTimersOut = {}

function debounceFlashClear(id, type, commit) {
  const timers = type === 'in' ? flashTimersIn : flashTimersOut
  if (timers[id]) {
    clearTimeout(timers[id])
  }
  timers[id] = setTimeout(() => {
    if (type === 'in') {
      commit('clearFlashIn', id)
    } else {
      commit('clearFlashOut', id)
    }
    delete timers[id]
  }, 100)
}

function setupWebsocket(id, commit) {
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
    commit('setConnectionStatus', { id, status: 'connected' })
  }

  socket.onmessage = event => {
    const message = JSON.parse(event.data)
    commit('setMessage', { id, message })
    commit('setLastIncomingActivity', { id, timestamp: Date.now() })
    commit('setFlashIn', { id, value: true })
    debounceFlashClear(id, 'in', commit)
  }

  socket.onclose = e => {
    console.log(
      `WebSocket ${id} closed. Reconnecting in 2 seconds...`,
      e.reason,
    )
    commit('setConnectionStatus', { id, status: 'disconnected' })
    delete webSockets[id] // Remove reference before reconnecting
    setTimeout(() => {
      setupWebsocket(id, commit)
    }, 2000)
  }

  socket.onerror = error => {
    console.error(`WebSocket ${id} encountered error`, error)
    commit('setConnectionStatus', { id, status: 'disconnected' })
    socket.close()
  }

  // Wrap send to track outgoing data activity
  const originalSend = socket.send;
  socket.send = function (data) {
    if (this.readyState !== WebSocket.OPEN) {
      console.warn(`WebSocket [${id}] is not open. Current state: ${this.readyState}`);
      return;
    }
    commit('setLastOutgoingActivity', { id, timestamp: Date.now() });
    commit('setFlashOut', { id, value: true });
    debounceFlashClear(id, 'out', commit);
    return originalSend.call(this, data);
  };

  webSockets[id] = socket // Store reference
}

const state = {
  messages: {}, // Store messages per WebSocket ID
  connectionStatus: {},
  lastIncomingActivity: {},
  lastOutgoingActivity: {},
  flashIn: {},
  flashOut: {},
}

const mutations = {
  setMessage(state, { id, message }) {
    state.messages[id] = message
  },
  setConnectionStatus(state, { id, status }) {
    state.connectionStatus[id] = status
  },
  setFlashIn(state, { id, value }) {
    state.flashIn[id] = value
  },
  setFlashOut(state, { id, value }) {
    state.flashOut[id] = value
  },
  clearFlashIn(state, id) {
    state.flashIn[id] = false
  },
  clearFlashOut(state, id) {
    state.flashOut[id] = false
  },
  setLastIncomingActivity(state, { id, timestamp }) {
    state.lastIncomingActivity[id] = timestamp
  },
  setLastOutgoingActivity(state, { id, timestamp }) {
    state.lastOutgoingActivity[id] = timestamp
  },
}


const getters = {
  isFlashingIn: (state) => (id) => state.flashIn[id] || false,
  isFlashingOut: (state) => (id) => state.flashOut[id] || false,
}

const actions = {
  sendMessage({}, { id, message }) {
    // console.log(id)
    // trashed "commit" to avoid ts warning
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
    // console.log('sent ' + id)
  },

  setupWebSocket({ commit }, id) {
    setupWebsocket(id, commit) // Pass unique ID
  },

  closeWebSocket({}, id) {
    if (webSockets[id]) {
      webSockets[id].close()
      delete webSockets[id] // Cleanup
    }
  },
}

export default {
  namespaced: true,
  state,
  getters,
  mutations,
  actions,
}

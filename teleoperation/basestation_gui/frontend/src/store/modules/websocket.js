const webSockets = {} // Store WebSocket instances dynamically

function setupWebsocket(id, commit) {
  if (webSockets[id]) {
    console.warn(`WebSocket with ID ${id} already exists.`)
    return
  }

  const socket = new WebSocket(`ws://localhost:8000/ws/${id}`)

  socket.onopen = () => {
    console.log(`WebSocket ${id} Connected`)
  }

  socket.onmessage = event => {
    const message = JSON.parse(event.data)
    commit('setMessage', { id, message }) // Store messages per WebSocket ID
  }

  socket.onclose = e => {
    console.log(`WebSocket ${id} closed. Reconnecting in 2 seconds...`, e.reason)
    delete webSockets[id] // Remove reference before reconnecting
    setTimeout(() => {
      setupWebsocket(id, commit)
    }, 2000)
  }

  socket.onerror = error => {
    console.error(`WebSocket ${id} encountered error`, error)
    socket.close()
  }

  webSockets[id] = socket // Store reference
}

const state = {
  messages: {} // Store messages per WebSocket ID
}

const mutations = {
  setMessage(state, { id, message }) {
    state.messages[id] = message
  }
}

const actions = {
  sendMessage({ commit }, { id, message }) {
    console.log(webSockets, id)
    const socket = webSockets[id]
    console.log("sending message to ", socket)
    if (!socket || socket.readyState !== WebSocket.OPEN) return
    socket.send(JSON.stringify(message))
  },

  setupWebSocket({ commit }, id) {
    setupWebsocket(id, commit) // Pass unique ID
  },

  closeWebSocket({ commit }, id) {
    if (webSockets[id]) {
      webSockets[id].close()
      delete webSockets[id] // Cleanup
    }
  }
}

export default {
  namespaced: true,
  state,
  mutations,
  actions
}

const webSockets = {} // Store WebSocket instances dynamically

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
  }

  socket.onmessage = event => {
    const message = JSON.parse(event.data)
    commit('setMessage', { id, message })
  }

  socket.onclose = e => {
    console.log(
      `WebSocket ${id} closed. Reconnecting in 2 seconds...`,
      e.reason,
    )
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
  messages: {}, // Store messages per WebSocket ID
}

const mutations = {
  setMessage(state, { id, message }) {
    console.log("setMessage received: ", message)
    state.messages[id] = message
  },
}

const actions = {
  sendMessage({  }, { id, message }) { // trashed "commit" to avoid ts warning
    console.log('webSockets:', webSockets, 'id:', id, 'message:', message);
    const socket = webSockets[id]
    if(!socket){
      console.log('websocket selection failed with id "',id,'"')
      return
    }
    if (!socket.readyState) {
      console.log('websocket ' + id + ' not ready')
      return
    }
    socket.send(JSON.stringify(message))
    console.log('sent ' + id)
  },

  setupWebSocket({ commit }, id) {
    setupWebsocket(id, commit) // Pass unique ID
  },

  closeWebSocket({  }, id) {
    if (webSockets[id]) {
      webSockets[id].close()
      delete webSockets[id] // Cleanup
    }
  },
}

export default {
  namespaced: true,
  state,
  mutations,
  actions,
}

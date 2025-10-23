// initial state
const state = {
  notifications: [],
  nextId: 1
}

// getters
const getters = {
  notifications: (state) => state.notifications,
  unreadCount: (state) => state.notifications.filter(n => !n.read).length,
  hasUnread: (state) => state.notifications.some(n => !n.read)
}

// mutations
const mutations = {
  addNotification(state, payload) {
    const notification = {
      id: state.nextId++,
      timestamp: new Date().toISOString(),
      component: payload.component || 'Unknown',
      errorType: payload.errorType || 'error',
      message: payload.message || 'Unknown error',
      fullData: payload.fullData || {},
      read: false
    }
    state.notifications.unshift(notification)
  },

  markAsRead(state, notificationId) {
    const notification = state.notifications.find(n => n.id === notificationId)
    if (notification) {
      notification.read = true
    }
  },

  markAllAsRead(state) {
    state.notifications.forEach(n => n.read = true)
  },

  removeNotification(state, notificationId) {
    const index = state.notifications.findIndex(n => n.id === notificationId)
    if (index !== -1) {
      state.notifications.splice(index, 1)
    }
  },

  clearAll(state) {
    state.notifications = []
  }
}

export default {
  namespaced: true,
  state,
  getters,
  mutations
}

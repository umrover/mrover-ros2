import { defineStore } from 'pinia'
import { ref, computed, watch } from 'vue'

export interface Notification {
  id: number
  timestamp: string
  component: string
  message: string
  fullData: unknown
  read: boolean
}

interface StoredState {
  notifications: Notification[]
  nextId: number
}

const STORAGE_KEY = 'notifications_store'
const MAX_NOTIFICATIONS = 100

function loadFromStorage(): StoredState {
  try {
    const stored = localStorage.getItem(STORAGE_KEY)
    if (stored) {
      return JSON.parse(stored)
    }
  } catch {}
  return { notifications: [], nextId: 1 }
}

function saveToStorage(state: StoredState) {
  localStorage.setItem(STORAGE_KEY, JSON.stringify(state))
}

export const useNotificationsStore = defineStore('notifications', () => {
  const stored = loadFromStorage()

  const notifications = ref<Notification[]>(stored.notifications)
  const nextId = ref(stored.nextId)

  watch([notifications, nextId], () => {
    saveToStorage({ notifications: notifications.value, nextId: nextId.value })
  }, { deep: true })

  const unreadCount = computed(() => notifications.value.filter(n => !n.read).length)
  const hasUnread = computed(() => notifications.value.some(n => !n.read))

  function addNotification(payload: {
    component?: string
    message?: string
    fullData?: unknown
  }) {
    const notification: Notification = {
      id: nextId.value++,
      timestamp: new Date().toISOString(),
      component: payload.component || 'Unknown',
      message: payload.message || 'Unknown error',
      fullData: payload.fullData || {},
      read: false
    }
    notifications.value.unshift(notification)
    if (notifications.value.length > MAX_NOTIFICATIONS) {
      notifications.value.length = MAX_NOTIFICATIONS
    }
  }

  function markAllAsRead() {
    notifications.value.forEach(n => n.read = true)
  }

  function removeNotification(notificationId: number) {
    const index = notifications.value.findIndex(n => n.id === notificationId)
    if (index !== -1) {
      notifications.value.splice(index, 1)
    }
  }

  function clearAll() {
    notifications.value = []
  }

  function addAPIError(endpoint: string, message: string, status: number) {
    addNotification({
      component: endpoint,
      message,
      fullData: { endpoint, status, message }
    })
  }

  return {
    notifications,
    unreadCount,
    hasUnread,
    addNotification,
    addAPIError,
    markAllAsRead,
    removeNotification,
    clearAll
  }
})

import { defineStore } from 'pinia'
import { ref, computed } from 'vue'

export interface Notification {
  id: number
  timestamp: string
  component: string
  errorType: string
  message: string
  fullData: unknown
  read: boolean
}

export const useNotificationsStore = defineStore('notifications', () => {
  // State
  const notifications = ref<Notification[]>([])
  const nextId = ref(1)

  // Getters
  const unreadCount = computed(() => notifications.value.filter(n => !n.read).length)
  const hasUnread = computed(() => notifications.value.some(n => !n.read))

  // Actions
  function addNotification(payload: {
    component?: string
    errorType?: string
    message?: string
    fullData?: unknown
  }) {
    const notification: Notification = {
      id: nextId.value++,
      timestamp: new Date().toISOString(),
      component: payload.component || 'Unknown',
      errorType: payload.errorType || 'error',
      message: payload.message || 'Unknown error',
      fullData: payload.fullData || {},
      read: false
    }
    notifications.value.unshift(notification)
  }

  function markAsRead(notificationId: number) {
    const notification = notifications.value.find(n => n.id === notificationId)
    if (notification) {
      notification.read = true
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

  return {
    // State
    notifications,
    // Getters
    unreadCount,
    hasUnread,
    // Actions
    addNotification,
    markAsRead,
    markAllAsRead,
    removeNotification,
    clearAll
  }
})

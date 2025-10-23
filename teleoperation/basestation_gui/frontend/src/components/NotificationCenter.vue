<template>
  <div class="notification-center">
    <button
      class="border border-2 rounded p-1 bg-transparent position-relative d-flex flex-column align-items-center justify-content-center"
      style="width: 50px; height: 50px; color: inherit; cursor: pointer;"
      @click="togglePanel"
    >
      <i class="bi bi-bell-fill" style="font-size: 1.25rem;"></i>
      <span
        v-if="unreadCount > 0"
        class="position-absolute top-0 start-100 translate-middle badge rounded-pill bg-danger"
      >
        {{ unreadCount }}
      </span>
    </button>

    <div v-if="showPanel" class="notification-panel">
      <div class="panel-header d-flex justify-content-between align-items-center p-3">
        <h5 class="m-0">Notifications</h5>
        <div class="d-flex gap-2">
          <button
            v-if="notifications.length > 0"
            class="btn btn-sm btn-outline-secondary"
            @click="markAllAsRead"
          >
            Mark All Read
          </button>
          <button
            v-if="notifications.length > 0"
            class="btn btn-sm btn-outline-danger"
            @click="clearAll"
          >
            Clear All
          </button>
          <button class="btn btn-sm btn-close" @click="showPanel = false"></button>
        </div>
      </div>

      <div class="panel-body">
        <div v-if="notifications.length === 0" class="text-center text-muted p-4">
          No notifications
        </div>

        <div
          v-for="notification in notifications"
          :key="notification.id"
          class="notification-item"
          :class="{ unread: !notification.read }"
        >
          <div class="notification-header d-flex justify-content-between align-items-start">
            <div class="flex-grow-1">
              <div class="d-flex align-items-center gap-2">
                <span class="badge bg-danger">{{ notification.errorType }}</span>
                <strong>{{ notification.component }}</strong>
                <span class="text-muted small">
                  {{ formatTimestamp(notification.timestamp) }}
                </span>
              </div>
              <p class="mb-1 mt-2">{{ notification.message }}</p>
            </div>
            <button
              class="btn btn-sm btn-link text-danger p-0"
              @click="removeNotification(notification.id)"
              title="Remove"
            >
              <i class="bi bi-x-lg"></i>
            </button>
          </div>

          <div class="notification-actions mt-2">
            <button
              class="btn btn-sm btn-link p-0"
              @click="toggleDetails(notification.id)"
            >
              <i
                :class="
                  expandedNotifications.includes(notification.id)
                    ? 'bi bi-chevron-up'
                    : 'bi bi-chevron-down'
                "
              ></i>
              {{ expandedNotifications.includes(notification.id) ? 'Hide' : 'Show' }} Details
            </button>
            <button
              v-if="!notification.read"
              class="btn btn-sm btn-link p-0 ms-3"
              @click="markAsRead(notification.id)"
            >
              Mark as Read
            </button>
          </div>

          <div
            v-if="expandedNotifications.includes(notification.id)"
            class="notification-details mt-2"
          >
            <pre class="json-display">{{ JSON.stringify(notification.fullData, null, 2) }}</pre>
          </div>
        </div>
      </div>
    </div>

    <div v-if="showPanel" class="notification-overlay" @click="showPanel = false"></div>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import { useNotificationsStore } from '@/stores/notifications'

export default defineComponent({
  setup() {
    const notificationsStore = useNotificationsStore()
    return { notificationsStore }
  },

  data() {
    return {
      showPanel: false,
      expandedNotifications: [] as number[]
    }
  },

  computed: {
    notifications() {
      return this.notificationsStore.notifications
    },
    unreadCount() {
      return this.notificationsStore.unreadCount
    }
  },

  methods: {
    markAsRead(id: number) {
      this.notificationsStore.markAsRead(id)
    },
    markAllAsRead() {
      this.notificationsStore.markAllAsRead()
    },
    removeNotification(id: number) {
      this.notificationsStore.removeNotification(id)
    },
    clearAll() {
      this.notificationsStore.clearAll()
    },

    togglePanel() {
      this.showPanel = !this.showPanel
    },

    toggleDetails(notificationId: number) {
      const index = this.expandedNotifications.indexOf(notificationId)
      if (index > -1) {
        this.expandedNotifications.splice(index, 1)
      } else {
        this.expandedNotifications.push(notificationId)
      }
    },

    formatTimestamp(timestamp: string): string {
      const date = new Date(timestamp)
      const now = new Date()
      const diff = now.getTime() - date.getTime()

      // Less than 1 minute
      if (diff < 60000) {
        return 'Just now'
      }
      // Less than 1 hour
      if (diff < 3600000) {
        const minutes = Math.floor(diff / 60000)
        return `${minutes} min${minutes > 1 ? 's' : ''} ago`
      }
      // Less than 1 day
      if (diff < 86400000) {
        const hours = Math.floor(diff / 3600000)
        return `${hours} hour${hours > 1 ? 's' : ''} ago`
      }

      // Format as date
      return date.toLocaleString()
    }
  }
})
</script>

<style scoped>
.notification-center {
  position: relative;
}

.notification-panel {
  position: absolute;
  top: 100%;
  right: 0;
  margin-top: 10px;
  width: 500px;
  max-height: 600px;
  background-color: white;
  border-radius: 8px;
  box-shadow: 0 4px 20px rgba(0, 0, 0, 0.15);
  z-index: 1001;
  display: flex;
  flex-direction: column;
}

.notification-overlay {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  z-index: 1000;
}

.panel-header {
  border-bottom: 1px solid #dee2e6;
  background-color: #f8f9fa;
  border-radius: 8px 8px 0 0;
}

.panel-body {
  overflow-y: auto;
  max-height: 520px;
}

.notification-item {
  padding: 16px;
  border-bottom: 1px solid #dee2e6;
  transition: background-color 0.2s;
}

.notification-item:hover {
  background-color: #f8f9fa;
}

.notification-item.unread {
  background-color: #e7f3ff;
  border-left: 4px solid #0d6efd;
}

.notification-item.unread:hover {
  background-color: #d4e9ff;
}

.notification-item:last-child {
  border-bottom: none;
}

.json-display {
  background-color: #f8f9fa;
  border: 1px solid #dee2e6;
  border-radius: 4px;
  padding: 12px;
  font-size: 12px;
  overflow-x: auto;
  max-height: 300px;
  margin: 0;
}

.badge.rounded-pill {
  font-size: 10px;
  padding: 4px 6px;
}
</style>

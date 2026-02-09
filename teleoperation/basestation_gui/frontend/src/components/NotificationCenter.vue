<template>
  <div class="position-relative d-flex align-self-stretch">
    <button
      class="notification-btn border border-2 rounded"
      data-testid="pw-notification-bell"
      @click="togglePanel"
    >
      <i class="bi bi-bell-fill"></i>
      <span
        v-if="unreadCount > 0"
        class="notification-badge"
      >
        {{ unreadCount }}
      </span>
    </button>

    <div v-if="showPanel" class="notification-panel cmd-panel" data-testid="pw-notification-panel">
      <div class="d-flex justify-content-between align-items-center p-3 border-bottom border-2">
        <h4 class="component-header m-0">Notifications</h4>
        <div class="d-flex gap-2">
          <button
            v-if="notifications.length > 0"
            class="btn btn-sm btn-outline-danger border-2"
            data-testid="pw-notification-clear-all"
            @click="clearAll"
          >
            Clear All
          </button>
          <button class="btn btn-sm btn-outline-secondary border-2 cmd-btn-icon-sm" @click="showPanel = false">
            <i class="bi bi-x-lg"></i>
          </button>
        </div>
      </div>

      <div class="notification-list overflow-auto">
        <div v-if="notifications.length === 0" class="text-center text-muted p-4 notification-empty">
          No notifications
        </div>

        <div
          v-for="notification in notifications"
          :key="notification.id"
          class="p-3 border-bottom border-2 notification-item"
        >
          <div class="d-flex justify-content-between align-items-start">
            <div class="flex-grow-1">
              <div class="d-flex align-items-center gap-2">
                <span class="notification-component">{{ notification.component }}</span>
                <span class="notification-time text-muted">{{ formatTimestamp(notification.timestamp) }}</span>
              </div>
              <p class="mt-2 mb-1">{{ notification.message }}</p>
            </div>
            <button
              class="btn btn-sm btn-outline-secondary border-2 cmd-btn-icon-sm"
              @click="removeNotification(notification.id)"
              title="Remove"
            >
              <i class="bi bi-x-lg"></i>
            </button>
          </div>

          <div class="mt-2">
            <button
              class="btn btn-sm btn-outline-control border-2"
              @click="toggleDetails(notification.id)"
            >
              <i :class="expandedNotifications.includes(notification.id) ? 'bi bi-chevron-up' : 'bi bi-chevron-down'"></i>
              {{ expandedNotifications.includes(notification.id) ? 'Hide' : 'Show' }} Details
            </button>
          </div>

          <pre
            v-if="expandedNotifications.includes(notification.id)"
            class="notification-details p-2 mt-2 rounded overflow-auto"
          >{{ JSON.stringify(notification.fullData, null, 2) }}</pre>
        </div>
      </div>
    </div>

    <div v-if="showPanel" class="position-fixed top-0 start-0 w-100 h-100" style="z-index: 1050;" @click="showPanel = false"></div>
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
    removeNotification(id: number) {
      this.notificationsStore.removeNotification(id)
    },
    clearAll() {
      this.notificationsStore.clearAll()
    },

    togglePanel() {
      this.showPanel = !this.showPanel
      if (this.showPanel) {
        this.notificationsStore.markAllAsRead()
      }
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

      if (diff < 60000) {
        return 'Just now'
      }
      if (diff < 3600000) {
        const minutes = Math.floor(diff / 60000)
        return `${minutes} min${minutes > 1 ? 's' : ''} ago`
      }
      if (diff < 86400000) {
        const hours = Math.floor(diff / 3600000)
        return `${hours} hour${hours > 1 ? 's' : ''} ago`
      }

      return date.toLocaleString()
    }
  }
})
</script>

<style scoped>
.notification-btn {
  min-width: 48px;
  height: 100%;
  padding: 0.25rem 0.5rem;
  position: relative;
  background-color: var(--card-bg);
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  cursor: pointer;
}

.notification-btn i {
  font-size: 1.25rem;
}

.notification-badge {
  position: absolute;
  top: -4px;
  right: -4px;
  min-width: 18px;
  height: 18px;
  padding: 0 4px;
  font-size: 0.6875rem;
  font-weight: 600;
  line-height: 18px;
  text-align: center;
  color: #ffffff;
  background-color: var(--bs-danger);
  border-radius: 50rem;
}

.notification-panel {
  position: absolute;
  right: 0;
  top: 100%;
  margin-top: 0.5rem;
  width: 500px;
  z-index: 1051;
  box-shadow: var(--cmd-shadow-md);
}

.notification-list {
  max-height: 520px;
}

.notification-empty {
  font-size: 0.875rem;
}

.notification-item:last-child {
  border-bottom: none !important;
}

.notification-component {
  font-size: 0.8125rem;
  font-weight: 600;
}

.notification-time {
  font-size: 0.6875rem;
}

.notification-details {
  background-color: var(--view-bg);
  border: 2px solid var(--cmd-panel-border);
  font-size: 0.75rem;
  max-height: 300px;
}
</style>

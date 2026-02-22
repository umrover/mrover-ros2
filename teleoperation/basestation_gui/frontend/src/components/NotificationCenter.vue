<template>
  <div class="position-relative">
    <button
      class="btn border border-2 rounded position-relative d-flex align-items-center justify-content-center"
      style="width: 50px; height: 50px;"
      @click="togglePanel"
    >
      <i class="bi bi-bell-fill fs-5"></i>
      <span
        v-if="unreadCount > 0"
        class="position-absolute top-0 start-100 translate-middle badge rounded-pill bg-danger"
      >
        {{ unreadCount }}
      </span>
    </button>

    <div v-if="showPanel" class="dropdown-menu show position-absolute end-0 mt-2 shadow" style="width: 500px; z-index: 1051;">
      <div class="d-flex justify-content-between align-items-center p-3 border-bottom bg-light">
        <h5 class="m-0">Notifications</h5>
        <div class="d-flex gap-2">
          <button
            v-if="notifications.length > 0"
            class="btn btn-sm btn-outline-danger"
            @click="clearAll"
          >
            Clear All
          </button>
          <button class="btn-close" @click="showPanel = false"></button>
        </div>
      </div>

      <div class="overflow-auto" style="max-height: 520px;">
        <div v-if="notifications.length === 0" class="text-center text-muted p-4">
          No notifications
        </div>

        <div
          v-for="notification in notifications"
          :key="notification.id"
          class="p-3 border-bottom"
        >
          <div class="d-flex justify-content-between align-items-start">
            <div class="flex-grow-1">
              <div class="d-flex align-items-center gap-2">
                <strong>{{ notification.component }}</strong>
                <small class="text-muted">{{ formatTimestamp(notification.timestamp) }}</small>
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

          <div class="mt-2">
            <button
              class="btn btn-sm btn-link p-0"
              @click="toggleDetails(notification.id)"
            >
              <i :class="expandedNotifications.includes(notification.id) ? 'bi bi-chevron-up' : 'bi bi-chevron-down'"></i>
              {{ expandedNotifications.includes(notification.id) ? 'Hide' : 'Show' }} Details
            </button>
          </div>

          <pre
            v-if="expandedNotifications.includes(notification.id)"
            class="bg-light border rounded p-2 mt-2 mb-0 small overflow-auto"
            style="max-height: 300px;"
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

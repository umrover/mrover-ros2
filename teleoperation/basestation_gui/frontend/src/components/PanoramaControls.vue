<template>
  <div class="flex flex-col gap-2 w-full h-full">
    <h4 class="component-header">Panorama</h4>
    <button :class="['cmd-btn', buttonClass]" :disabled="loading" data-testid="pw-panorama-toggle" @click="toggle">
      <span class="inline-flex items-center gap-2">
        <span>{{ buttonText }}</span>
        <span v-if="loading" class="cmd-spinner cmd-spinner-sm"></span>
      </span>
    </button>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed } from 'vue'
import { chassisAPI } from '@/utils/chassisAPI'
import { useNotificationsStore } from '@/stores/notifications'

const notificationsStore = useNotificationsStore()

const capturing = ref(false)
const loading = ref(false)

const buttonText = computed(() => {
  if (loading.value) {
    return capturing.value ? 'Stopping...' : 'Starting...'
  }
  return capturing.value ? 'Stop' : 'Start'
})

const buttonClass = computed(() => {
  if (loading.value) return 'cmd-btn-warning'
  return capturing.value ? 'cmd-btn-danger' : 'cmd-btn-primary'
})

async function toggle() {
  loading.value = true
  try {
    if (capturing.value) {
      const response = await chassisAPI.stopPanorama()
      if (response.status === 'error') {
        notificationsStore.addNotification({
          component: 'Panorama',
          message: response.message || 'Failed to stop panorama',
          fullData: response
        })
      }
      capturing.value = false
    } else {
      const response = await chassisAPI.startPanorama()
      if (response.status === 'error') {
        notificationsStore.addNotification({
          component: 'Panorama',
          message: response.message || 'Failed to start panorama',
          fullData: response
        })
      } else {
        capturing.value = true
      }
    }
  } catch (err) {
    notificationsStore.addNotification({
      component: 'Panorama',
      message: 'Panorama request failed',
      fullData: err
    })
  } finally {
    loading.value = false
  }
}
</script>

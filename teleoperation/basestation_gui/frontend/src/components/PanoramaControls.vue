<template>
  <div class="flex flex-col gap-2 w-full h-full">
    <h4 class="component-header">Panorama</h4>
    <div class="flex flex-col gap-2">
      <button class="btn btn-primary " :disabled="startLoading" data-testid="pw-panorama-start" @click="start">
        <span class="inline-flex items-center gap-2">
          <span>{{ startLoading ? 'Starting...' : 'Start' }}</span>
          <span v-if="startLoading" class="spinner spinner-sm"></span>
        </span>
      </button>
      <button class="btn btn-danger " :disabled="stopLoading" data-testid="pw-panorama-stop" @click="stop">
        <span class="inline-flex items-center gap-2">
          <span>{{ stopLoading ? 'Stopping...' : 'Stop' }}</span>
          <span v-if="stopLoading" class="spinner spinner-sm"></span>
        </span>
      </button>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref } from 'vue'
import { chassisAPI } from '@/utils/chassisAPI'
import { useNotificationsStore } from '@/stores/notifications'

const notificationsStore = useNotificationsStore()

const startLoading = ref(false)
const stopLoading = ref(false)

async function start() {
  startLoading.value = true
  try {
    const response = await chassisAPI.startPanorama()
    if (response.status === 'error') {
      notificationsStore.addNotification({
        component: 'Panorama',
        message: response.message || 'Failed to start panorama',
        fullData: response
      })
    }
  } catch (err) {
    notificationsStore.addNotification({
      component: 'Panorama',
      message: 'Panorama start request failed',
      fullData: err
    })
  } finally {
    startLoading.value = false
  }
}

async function stop() {
  stopLoading.value = true
  try {
    const response = await chassisAPI.stopPanorama()
    if (response.status === 'error') {
      notificationsStore.addNotification({
        component: 'Panorama',
        message: response.message || 'Failed to stop panorama',
        fullData: response
      })
    }
  } catch (err) {
    notificationsStore.addNotification({
      component: 'Panorama',
      message: 'Panorama stop request failed',
      fullData: err
    })
  } finally {
    stopLoading.value = false
  }
}
</script>

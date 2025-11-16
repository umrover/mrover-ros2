<template>
  <div class="wrap p-2 flex-column justify-content-between">
    <h3 class="m-0 p-0">Panorama</h3>
    <div class="btn-group m-0 p-0" role="group" aria-label="Pano Controls">
      <button
        class="btn btn-success"
        :disabled="panoActive"
        :class="{ disabled: panoActive }"
        @click="togglePano('start')"
      >
        Start
      </button>
      <button
        class="btn btn-danger"
        :disabled="!panoActive"
        :class="{ disabled: !panoActive }"
        @click="togglePano('stop')"
      >
        Stop
      </button>
    </div>
  </div>
</template>

<script lang="ts">
import { mastAPI } from '@/utils/api'

export default {
  data() {
    return {
      panoActive: false,
    }
  },
  methods: {
    async togglePano(action: 'start' | 'stop') {
      this.panoActive = action === 'start'

      try {
        if (action === 'start') {
          await mastAPI.startPanorama()
        } else {
          const result = await mastAPI.stopPanorama()
          if (result.status === 'success' && result.image_path) {
            console.log('Panorama saved to:', result.image_path)
          }
        }
      } catch (error) {
        console.error('Failed to toggle panorama:', error)
        // Revert state on error
        this.panoActive = !this.panoActive
      }
    },
  },
}
</script>

<style scoped>
button:disabled {
  cursor: not-allowed;
}
.wrap {
  display: inline-flex;
  flex-direction: column;
  width: auto;
}
</style>

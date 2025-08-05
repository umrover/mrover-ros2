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
import Vuex from 'vuex'
const { mapActions } = Vuex

export default {
  data() {
    return {
      panoActive: false,
    }
  },
  methods: {
    ...mapActions('websocket', ['sendMessage']),
    togglePano(action: 'start' | 'stop') {
      this.panoActive = action === 'start'
      this.sendMessage({
        id: 'mast',
        message: {
          type: 'pano',
          action,
        },
      })
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

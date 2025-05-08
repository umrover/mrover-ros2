<template>
  <div class="wrap flex flex-col items-center justify-center p-4">
    <div class="d-flex">
      <h3 class="m-0 me-3">Pano Cam</h3>
      <div class="button-group">
        <button 
          v-if="!panoActive" 
          class="btn btn-success" @click="togglePano">
          Start
        </button>
        <button 
          v-if="panoActive" 
          class="btn btn-danger" @click="togglePano">
          Stop
        </button>
      </div>
    </div>
  </div>
</template>

<script lang="ts">
import { mapActions } from 'vuex';

export default {

  data() {
    return {
      panoActive: false
    };
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    togglePano() {
      if (this.panoActive) {
        this.sendMessage({ type: 'pano', action: 'stop' });
      } else {
        this.sendMessage({ type: 'pano', action: 'start' });
      }
      this.panoActive = !this.panoActive;
    }
  }
};
</script>

<style scoped>
.wrap {
  display: flex;
  flex-direction: column;
  height: 100%;
  width: 100%;
  text-align: center;
}

.pano-btn {
  padding: 10px 20px;
  color: white;
}
</style>

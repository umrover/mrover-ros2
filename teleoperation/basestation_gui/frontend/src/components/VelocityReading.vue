<template>
  <div class="d-flex flex-column gap-2">
    <div class="d-flex align-items-baseline justify-content-between">
      <span class="cmd-data-label">Linear</span>
      <div>
        <span class="cmd-data-value velocity-value">{{ linear_x.toFixed(2) }}</span>
        <span class="cmd-data-unit ms-1">m/s</span>
      </div>
    </div>
    <div class="d-flex align-items-baseline justify-content-between">
      <span class="cmd-data-label">Angular</span>
      <div>
        <span class="cmd-data-value velocity-value">{{ angular_z.toFixed(2) }}</span>
        <span class="cmd-data-unit ms-1">rad/s</span>
      </div>
    </div>
  </div>
</template>

<script lang="ts">
import type { WebSocketState } from '../types/websocket'
import Vuex from 'vuex'
const { mapState } = Vuex
export default {
  data() {
    return {
      linear_x: 0,
      angular_z: 0,
    }
  },

  computed: {
    ...mapState('websocket', {
      navMessage: (state: WebSocketState) => state.messages['nav']
    }),
  },

  watch: {
    navMessage(msg) { // NOT YET IMPLEMENTED / MISSING IMPLEMENTATION
      if (msg.type == 'cmd_vel') {
        this.linear_x = msg.linear.x
        this.angular_z = msg.angular.z
      }
    },
  },
}
</script>

<style scoped>
.velocity-value {
  font-family: var(--cmd-font-mono);
  font-size: 1rem;
}
</style>

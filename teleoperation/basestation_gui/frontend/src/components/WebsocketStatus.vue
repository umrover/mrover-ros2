<template>
  <div class="p-2 rounded border shadow bg-white text-body">
    <h5 class="fw-bold mb-2">WebSocket Status</h5>
    <div
      v-for="(status, id) in connectionStatus"
      :key="id"
      class="d-flex align-items-center gap-2 mb-1"
    >
      <span class="fw-bold">C</span>
      <span>{{ id }}</span>
      <span class="d-flex align-items-center gap-1">
			<span :class="[isFlashingOut(id) ? 'text-success' : 'text-secondary']">⬤</span>
			<span :class="[isFlashingIn(id) ? 'text-danger' : 'text-secondary']">⬤</span>
      </span>
    </div>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import Vuex from 'vuex'
const { mapState } = Vuex

export default defineComponent({
  name: 'WebSocketStatus',

  computed: {
    ...mapState('websocket', ['connectionStatus']),
  },

  methods: {
    isFlashingIn(id: string): boolean {
      return this.$store.getters['websocket/isFlashingIn'](id)
    },
    isFlashingOut(id: string): boolean {
      return this.$store.getters['websocket/isFlashingOut'](id)
    },
  },
})
</script>

<style scoped>
.blinking {
  animation: blink 0.1s linear;
}
@keyframes blink {
  50% {
    opacity: 0;
  }
}
</style>

<template>
  <div class="velocity-wrapper">
    <h4>Velocity</h4>
    <div class="velocity-row pb-1">
      <span class="label">Linear:</span>
      <span class="value">{{ linear_x.toFixed(3) }} m/s</span>
    </div>
    <div class="velocity-row">
      <span class="label">Angular:</span>
      <span class="value">{{ angular_z.toFixed(3) }} rad/s</span>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const linear_x = ref(0)
const angular_z = ref(0)

const navMessage = computed(() => messages.value['nav'])

watch(navMessage, (msg) => {
  if (msg && msg.type == 'cmd_vel') {
    linear_x.value = msg.linear.x
    angular_z.value = msg.angular.z
  }
})
</script>

<style scoped>
.velocity-wrapper {
  background-color: #dddddd;
  padding: 0.5rem;
  border-radius: 8px;
  max-width: 320px;
}

h3 {
  font-family: monospace;
  letter-spacing: -0.1rem;
}

.velocity-row {
  display: flex;
  flex-direction: column;
}

.label {
  font-weight: bold;
  letter-spacing: -0.03rem;
}
</style>

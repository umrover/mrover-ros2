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
import type { CmdVelMessage } from '@/types/websocket'

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const linear_x = ref(0)
const angular_z = ref(0)

const navMessage = computed(() => messages.value['nav'])

watch(navMessage, (msg) => {
  if (msg && (msg as CmdVelMessage).type == 'cmd_vel') {
    const cmdVelMsg = msg as CmdVelMessage;
    linear_x.value = cmdVelMsg.linear.x
    angular_z.value = cmdVelMsg.angular.z
  }
})
</script>

<style scoped>
.velocity-wrapper {
  background-color: var(--view-bg);
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

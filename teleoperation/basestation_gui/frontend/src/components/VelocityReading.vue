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
.velocity-value {
  font-family: var(--cmd-font-mono);
  font-size: 1rem;
}
</style>

<template>
  <div class="rounded d-flex flex-row gap-5 w-100 h-100 align-items-center">
    <h4 class="component-header">Velocity</h4>
    <div class="d-flex flex-column gap-1 flex-grow-1">
      <div class="d-flex align-items-baseline gap-1 text-nowrap">
        <span class="cmd-data-label">Linear</span>
        <span class="cmd-data-value velocity-value">{{ linear_x.toFixed(3) }}</span>
        <span class="cmd-data-unit">m/s</span>
      </div>
      <div class="d-flex align-items-baseline gap-1 text-nowrap">
        <span class="cmd-data-label">Angular</span>
        <span class="cmd-data-value velocity-value">{{ angular_z.toFixed(3) }}</span>
        <span class="cmd-data-unit">rad/s</span>
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

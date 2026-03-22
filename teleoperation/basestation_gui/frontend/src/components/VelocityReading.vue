<template>
  <div class="flex flex-col gap-2">
    <div class="flex items-baseline justify-between">
      <span class="cmd-data-label">Linear</span>
      <div>
        <span class="cmd-data-value text-base">{{ linear_x.toFixed(2) }}</span>
        <span class="cmd-data-unit ml-1">m/s</span>
      </div>
    </div>
    <div class="flex items-baseline justify-between">
      <span class="cmd-data-label">Angular</span>
      <div>
        <span class="cmd-data-value text-base">{{ angular_z.toFixed(2) }}</span>
        <span class="cmd-data-unit ml-1">rad/s</span>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import type { CmdVelMessage } from '@/types/websocket'

const websocketStore = useWebsocketStore()

const linear_x = ref(0)
const angular_z = ref(0)

websocketStore.onMessage<CmdVelMessage>('nav', 'cmd_vel', (msg) => {
  linear_x.value = msg.linear.x
  angular_z.value = msg.angular.z
})
</script>


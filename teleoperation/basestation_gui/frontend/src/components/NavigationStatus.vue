<template>
  <div class="d-flex flex-column gap-2 flex-fill">
    <div>
      <div
        v-if="!stuckStatus"
        class="island p-2 rounded bg-success text-center"
      >
        <h3 class="m-0 p-0">Nominal Conditions</h3>
      </div>
      <div v-else class="island p-2 rounded bg-danger text-center">
        <h3 class="m-0 p-0">Obstruction Detected</h3>
      </div>
    </div>
    <div
      :class="[
        'rounded p-2 flex-fill d-flex align-items-center justify-content-center',
        ledColor,
      ]"
    >
      <h3 class="m-0">Nav State: {{ navState }}</h3>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { useAutonomyStore } from '@/stores/autonomy'
import { storeToRefs } from 'pinia'

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const autonomyStore = useAutonomyStore()
const { teleopEnabled } = storeToRefs(autonomyStore)

const stuckStatus = ref(false)
const navState = ref('OffState')

const ledColor = computed(() => {
  if (teleopEnabled.value) return 'bg-primary'
  if (navState.value === 'DoneState') return 'blink'
  return 'bg-danger'
})

const navMessage = computed(() => messages.value['nav'])

watch(navMessage, (msg: unknown) => {
  if (typeof msg === 'object' && msg !== null && 'type' in msg) {
    const typedMsg = msg as { type: string; state?: string }
    if (typedMsg.type === 'nav_state') {
      navState.value = typedMsg.state || 'OffState'
    }
  }
})
</script>

<style>
.blink {
  animation: blink-green 1s infinite;
}

@keyframes blink-green {
  0%, 50% {
    background-color: var(--bs-success);
  }
  51%, 100% {
    background-color: transparent;
  }
}
</style>

<template>
  <div class="flex flex-col gap-2 flex-1">
    <div class="cmd-panel status-panel px-4 py-2" data-testid="pw-status-panel" :class="stuckStatus ? 'status-panel--error' : 'status-panel--ok'">
      <div class="flex items-center gap-2">
        <span class="status-dot rounded-full"></span>
        <span class="status-label">{{ stuckStatus ? 'Obstruction Detected' : 'Nominal Conditions' }}</span>
      </div>
    </div>
    <div class="cmd-panel nav-state-panel flex-1 flex items-center justify-center" data-testid="pw-nav-state-panel" :class="ledColorClass">
      <div class="flex flex-col items-center gap-1">
        <span class="cmd-data-label">Nav State</span>
        <span class="nav-state-value" data-testid="pw-nav-state-value">{{ navState }}</span>
      </div>
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

const ledColorClass = computed(() => {
  if (teleopEnabled.value) return 'nav-state--info'
  if (navState.value === 'DoneState') return 'nav-state--ok nav-state--blink'
  return 'nav-state--error'
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

<style scoped>
.status-dot {
  flex-shrink: 0;
  width: clamp(8px, 0.6vw, 12px);
  height: clamp(8px, 0.6vw, 12px);
}

.status-label {
  font-size: 0.875rem;
  font-weight: 600;
  text-transform: uppercase;
  letter-spacing: 0.03em;
}

.status-panel--ok {
  background-color: var(--cmd-status-ok);
  border-color: var(--cmd-status-ok);
}

.status-panel--ok .status-dot { background-color: #fff; }
.status-panel--ok .status-label { color: #fff; }

.status-panel--error {
  background-color: var(--cmd-status-error);
  border-color: var(--cmd-status-error);
}

.status-panel--error .status-dot { background-color: #fff; }
.status-panel--error .status-label { color: #fff; }

.nav-state-panel {
  min-height: clamp(70px, 5vw, 100px);
  transition: all var(--cmd-transition);
}

.nav-state-value {
  font-size: 1.5rem;
  font-weight: 700;
  letter-spacing: -0.02em;
}

.nav-state--error {
  background-color: var(--cmd-status-error);
  border-color: var(--cmd-status-error);
}

.nav-state--error .cmd-data-label { color: rgb(255 255 255 / 80%); }
.nav-state--error .nav-state-value { color: #fff; }

.nav-state--ok {
  background-color: var(--cmd-status-ok);
  border-color: var(--cmd-status-ok);
}

.nav-state--ok .cmd-data-label { color: rgb(255 255 255 / 80%); }
.nav-state--ok .nav-state-value { color: #fff; }

.nav-state--info {
  background-color: var(--cmd-accent);
  border-color: var(--cmd-accent);
}

.nav-state--info .cmd-data-label { color: rgb(255 255 255 / 80%); }
.nav-state--info .nav-state-value { color: #fff; }

.nav-state--blink .nav-state-value {
  animation: blink-text 1s infinite;
}

@keyframes blink-text {
  0%, 50% { opacity: 1; }
  51%, 100% { opacity: 0.3; }
}
</style>

<template>
  <div class="d-flex flex-column gap-2 flex-fill">
    <div class="cmd-panel status-panel px-3 py-2" data-testid="pw-status-panel" :class="stuckStatus ? 'status-panel--error' : 'status-panel--ok'">
      <div class="d-flex align-items-center gap-2">
        <span class="status-dot rounded-circle"></span>
        <span class="status-label">{{ stuckStatus ? 'Obstruction Detected' : 'Nominal Conditions' }}</span>
      </div>
    </div>
    <div class="cmd-panel nav-state-panel flex-fill d-flex align-items-center justify-content-center" data-testid="pw-nav-state-panel" :class="ledColorClass">
      <div class="d-flex flex-column align-items-center gap-1">
        <span class="cmd-data-label">Nav State</span>
        <span class="nav-state-value" data-testid="pw-nav-state-value">{{ navState }}</span>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const ledColorClass = ref('nav-state--error')
const stuckStatus = ref(false)
const navState = ref('OffState')

const navMessage = computed(() => messages.value['nav'])

watch(navMessage, (msg: unknown) => {
  if (typeof msg === 'object' && msg !== null && 'type' in msg) {
    const typedMsg = msg as { type: string; state?: string; color?: number }
    if (typedMsg.type === 'nav_state') {
      navState.value = typedMsg.state || 'OffState'
    } else if (typedMsg.type === 'led_color') {
      if (typedMsg.color === 0) {
        ledColorClass.value = 'nav-state--error'
      } else if (typedMsg.color === 1) {
        ledColorClass.value = 'nav-state--info'
      } else if (typedMsg.color === 2) {
        ledColorClass.value = 'nav-state--ok nav-state--blink'
      }
    }
  }
})
</script>

<style scoped>
.status-dot {
  width: clamp(8px, 0.6vw, 12px);
  height: clamp(8px, 0.6vw, 12px);
  flex-shrink: 0;
}

.status-label {
  font-size: 0.875rem;
  font-weight: 600;
  text-transform: uppercase;
  letter-spacing: 0.03em;
}

.status-panel--ok {
  background-color: var(--bs-success);
  border-color: var(--bs-success);
}

.status-panel--ok .status-dot { background-color: #fff; }
.status-panel--ok .status-label { color: #fff; }

.status-panel--error {
  background-color: var(--bs-danger);
  border-color: var(--bs-danger);
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
  background-color: var(--bs-danger);
  border-color: var(--bs-danger);
}

.nav-state--error .cmd-data-label { color: rgba(255, 255, 255, 0.8); }
.nav-state--error .nav-state-value { color: #fff; }

.nav-state--ok {
  background-color: var(--bs-success);
  border-color: var(--bs-success);
}

.nav-state--ok .cmd-data-label { color: rgba(255, 255, 255, 0.8); }
.nav-state--ok .nav-state-value { color: #fff; }

.nav-state--info {
  background-color: var(--bs-primary);
  border-color: var(--bs-primary);
}

.nav-state--info .cmd-data-label { color: rgba(255, 255, 255, 0.8); }
.nav-state--info .nav-state-value { color: #fff; }

.nav-state--blink .nav-state-value {
  animation: blink-text 1s infinite;
}

@keyframes blink-text {
  0%, 50% { opacity: 1; }
  51%, 100% { opacity: 0.3; }
}
</style>

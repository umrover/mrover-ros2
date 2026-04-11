<template>
  <div class="flex-1 flex">
    <div class="panel nav-state-panel flex-1 flex items-center justify-center" data-testid="pw-nav-state-panel" :class="ledColorClass">
      <div class="flex flex-col items-center gap-1">
        <span class="data-label">Nav State</span>
        <span class="nav-state-value" data-testid="pw-nav-state-value">{{ navState }}</span>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { computed } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { useAutonomyStore } from '@/stores/autonomy'
import { storeToRefs } from 'pinia'
import type { NavStateMessage } from '@/types/coordinates'

const websocketStore = useWebsocketStore()

const autonomyStore = useAutonomyStore()
const { teleopEnabled, navState } = storeToRefs(autonomyStore)

const ledColorClass = computed(() => {
  if (teleopEnabled.value) return 'nav-state--info'
  if (navState.value === 'DoneState') return 'nav-state--ok nav-state--blink'
  return 'nav-state--error'
})

websocketStore.onMessage<NavStateMessage>('nav', 'nav_state', (msg) => {
  autonomyStore.setNavState(msg.state || 'OffState')
})
</script>

<style scoped>
.nav-state-panel {
  min-height: clamp(70px, 5vw, 100px);
  transition: all var(--transition);
}

.nav-state-value {
  font-size: 1.5rem;
  font-weight: 700;
  letter-spacing: -0.02em;
}

.nav-state--error {
  background-color: var(--status-error);
  border-color: var(--status-error);
}

.nav-state--error .data-label { color: var(--text-on-status); opacity: 0.8; }
.nav-state--error .nav-state-value { color: var(--text-on-status); }

.nav-state--ok {
  background-color: var(--status-ok);
  border-color: var(--status-ok);
}

.nav-state--ok .data-label { color: var(--text-on-status); opacity: 0.8; }
.nav-state--ok .nav-state-value { color: var(--text-on-status); }

.nav-state--info {
  background-color: var(--accent);
  border-color: var(--accent);
}

.nav-state--info .data-label { color: var(--text-on-status); opacity: 0.8; }
.nav-state--info .nav-state-value { color: var(--text-on-status); }

.nav-state--blink {
  animation: blink-bg 1s infinite;
}

@keyframes blink-bg {
  0%, 50% { opacity: 1; }
  51%, 100% { opacity: 0.3; }
}
</style>

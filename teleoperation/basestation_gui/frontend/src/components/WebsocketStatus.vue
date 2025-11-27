<template>
  <div class="d-flex justify-content-center" style="user-select: none;">
    <div
      v-if="Object.keys(connectionStatus as Record<string, any>).length > 0"
      class="justify-content-center align-items-center border border-2 rounded px-1 me-1"
    >
      <div class="d-flex align-items-center gap-2">
        <IndicatorDot :is-active="true" />
        <span class="fw-semibold">= TX</span>
      </div>
      <div class="d-flex align-items-center gap-2">
        <IndicatorDot :is-active="false" />
        <span class="fw-semibold">= RX</span>
      </div>
    </div>
    <div class="gap-1 d-flex">
      <div
        v-for="(status, id) in connectionStatus"
        :key="id"
        :class="[
          'mx-0 flex-column align-items-center border border-2 rounded p-1',
          status === 'disconnected' ? 'bg-warning' : ''
        ]"
      >
        <p class="fw-bold m-0 p-0 text-center">{{ getAlias(id) }}</p>

        <div class="d-flex justify-content-center align-items-center gap-2">
          <div
            class="rounded-circle"
            :class="isFlashingOut(id) ? 'bg-success' : 'bg-secondary'"
            style="width: 16px; height: 16px"
          ></div>
          <div
            class="rounded-circle"
            :class="isFlashingIn(id) ? 'bg-danger' : 'bg-secondary'"
            style="width: 16px; height: 16px"
          ></div>
        </div>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import IndicatorDot from './IndicatorDot.vue'

const websocketStore = useWebsocketStore()
const { connectionStatus } = storeToRefs(websocketStore)
const { isFlashingIn, isFlashingOut } = websocketStore

const aliasMap: Record<string, string> = {
  arm: 'arm',
  auton: 'auton',
  drive: 'drive',
  mast: 'mast',
  nav: 'nav',
  science: 'sci',
  waypoints: 'wypt',
  latency: 'lat',
}

const getAlias = (id: string): string => {
  return aliasMap[id] || id
}
</script>

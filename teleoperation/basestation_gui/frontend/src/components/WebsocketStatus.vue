<template>
  <div class="d-flex justify-content-center" style="user-select: none;">
    <div
      v-if="Object.keys(connectionStatus as Record<string, any>).length > 0"
      class="justify-content-center align-items-center border border-2 rounded px-1 me-1"
    >
      <div class="d-flex align-items-center gap-2">
        <IndicatorDot :is-active="true" />
        <span class="ws-label">= TX</span>
      </div>
      <div class="d-flex align-items-center gap-2">
        <IndicatorDot :is-active="false" />
        <span class="ws-label">= RX</span>
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
        <p class="ws-connection-name m-0 p-0 text-center">{{ getAlias(id) }}</p>

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

<script lang="ts">
import { defineComponent } from 'vue'
import Vuex from 'vuex'
const { mapState } = Vuex

export default defineComponent({
  data() {
    return {
      aliasMap: {
        arm: 'arm',
        auton: 'auton',
        drive: 'drive',
        mast: 'mast',
        nav: 'nav',
        science: 'sci',
        waypoints: 'wypt',
      } as Record<string, string>,
    }
  },

const aliasMap: Record<string, string> = {
  arm: 'arm',
  drive: 'drv',
  chassis: 'cha',
  nav: 'nav',
  science: 'sci',
  latency: 'lat',
  recording: 'rec',
}

  methods: {
    isFlashingIn(id: string): boolean {
      return this.$store.getters['websocket/isFlashingIn'](id)
    },
    isFlashingOut(id: string): boolean {
      return this.$store.getters['websocket/isFlashingOut'](id)
    },
    getAlias(id: string): string {
      return this.aliasMap[id] || id
    },
  },
})
</script>

<style scoped>
.ws-label {
  font-weight: 600;
}

.ws-connection-name {
  font-weight: 700;
}
</style>

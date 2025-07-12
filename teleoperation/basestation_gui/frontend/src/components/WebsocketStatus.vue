<template>
  <div class="d-flex justify-content-center">
    <div
      v-for="(status, id) in connectionStatus"
      :key="id"
      class="mx-1 d-flex flex-column align-items-center border border-2 rounded p-1"
    >
      <p class="fw-bold m-0 p-0 text-center">{{ getAlias(id) }}</p>

      <div class="d-flex justify-content-center align-items-center gap-2">
        <div
          class="rounded-circle"
          :class="isFlashingOut(id) ? 'bg-success' : 'bg-secondary'"
          style="width: 16px; height: 16px;"
        ></div>
        <div
          class="rounded-circle"
          :class="isFlashingIn(id) ? 'bg-danger' : 'bg-secondary'"
          style="width: 16px; height: 16px;"
        ></div>
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
        'arm': 'arm',
        'auton': 'auton',
        'drive': 'drive',
        'mast': 'mast',
        'nav': 'nav',
        'science': 'sci',
        'waypoints': 'wypt',
      } as Record<string, string>,
    }
  },

  computed: {
    ...mapState('websocket', ['connectionStatus']),
  },

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
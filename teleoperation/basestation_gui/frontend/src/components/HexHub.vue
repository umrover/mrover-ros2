<template>
  <div class="p-2 d-flex">
    <div class="me-2 d-flex flex-column justify-content-between">
      <h3 class="m-0 p-0">HexHub</h3>
      <div class="btn-group d-flex justify-content-center">
          <button
              class="btn"
              :class="orientation === 'cw' ? 'btn-primary' : 'btn-outline-primary'"
              @click="setOrientation('cw')"
          >
              CW
          </button>
          <button
              class="btn"
              :class="orientation === 'ccw' ? 'btn-primary' : 'btn-outline-primary'"
              @click="setOrientation('ccw')"
          >
              CCW
          </button>
      </div>
    </div>
    <div class="btn-group d-flex" role="group">
      <label
        v-for="(name, i) in siteList"
        :key="i"
        class="btn flex-fill align-items-center d-flex justify-content-center lh-1"
        :class="currentSite === i ? 'btn-primary text-white' : 'btn-outline-primary'"
      >
        <input
          class="btn-check"
          v-model="currentSite"
          type="radio"
          :value="i"
          @change="emitSite"
          autocomplete="off"
        />
        {{ name }}
      </label>
    </div>
  </div>
</template>


<script lang="ts">
import Vuex from 'vuex'
const { mapState, mapActions } = Vuex

export default {
  emits: ['selectSite', 'orientation'],
  data() {
    return {
      currentSite: 0,
      siteList: ['Site A', 'Site B', 'Sample Cache', 'Empty Soil Deposit', 'Blocked'],
      orientation: 'cw',
    }
  },
  computed: {
    ...mapState('websocket', ['message']),
  },
  methods: {
    ...mapActions('websocket', ['sendMessage']),
    setOrientation(value: 'cw' | 'ccw') {
      this.orientation = value
      this.$emit('orientation', value === 'ccw')
    },
    emitSite() {
      this.$emit('selectSite', this.currentSite)
    },
  },
}
</script>

<style scoped>
.btn {
  width: 80px;
}
</style>
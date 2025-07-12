<template>
  <div>
    <h3 class="text-center">HexHub Controls</h3>

    <!-- Orientation toggle -->
    <div class="d-flex justify-content-center align-items-center my-3">
      <div class="btn-group" role="group" aria-label="Orientation Toggle">
        <button
          type="button"
          class="btn"
          :class="orientation === 'cw' ? 'btn-primary' : 'btn-outline-primary'"
          @click="setOrientation('cw')"
        >
          Clockwise
        </button>
        <button
          type="button"
          class="btn"
          :class="orientation === 'ccw' ? 'btn-primary' : 'btn-outline-primary'"
          @click="setOrientation('ccw')"
        >
          Counter-Clockwise
        </button>
      </div>
    </div>

    <!-- Site selection -->
    <div class="btn-group-vertical w-100" role="group" aria-label="Site Selection">
      <div
        v-for="(name, index) in siteList"
        :key="index"
        class="px-2"
      >
        <input
          v-model="currentSite"
          class="btn-check"
          type="radio"
          :id="'Site' + index"
          :value="index"
          @change="emitSite"
          autocomplete="off"
        />
        <label
          class="btn btn-outline-secondary w-100 my-1"
          :for="'Site' + index"
        >
          {{ name }}
        </label>
      </div>
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
      siteList: [
        'Site A',
        'Site B',
        'Sample Cache',
        'Empty Soil Deposit',
        'Blocked',
      ],
      orientation: 'cw', // 'cw' or 'ccw'
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
      console.log(this.currentSite)
    },
  },
}
</script>

<style scoped>

.grid {
  display: grid;
  grid-template-columns: repeat(2, auto);
  font-family: sans-serif;
  height: auto;
  align-items: center;
  justify-items: center;
}
</style>

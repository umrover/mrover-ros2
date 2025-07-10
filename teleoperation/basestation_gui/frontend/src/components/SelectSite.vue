<template>
  <!-- TODO: Add in conditonal buttons based on which buttons are clicked already -->
  <!-- we want to rotate the hex hub in one direction? -->
  <div>
    <h2>Select Site</h2>
    <div class="form-check form-check-inline">
      <input
        v-model="site"
        class="form-check-input"
        type="radio"
        name="flexRadioDefault"
        id="site0"
        value="0"
      />
      <label class="form-check-label" for="site0"> Site A </label>
    </div>
    <div class="form-check form-check-inline">
      <input
        v-model="site"
        class="form-check-input"
        type="radio"
        name="flexRadioDefault"
        id="site1"
        value="1"
      />
      <label class="form-check-label" for="site1"> Site B </label>
    </div>
  </div>
</template>

<script lang="ts">
import type { WebSocketState } from '../types/websocket'
import Vuex from 'vuex'
const { mapState } = Vuex;

export default {
  components: {},
  data() {
    return {
      site: 0,
      autoShutdownEnabled: true,
    }
  },

  computed: {
    ...mapState('websocket', {
      scienceMessage: (state: WebSocketState) => state.messages['science']
    })
  },

  watch: {
    site(event) {
      this.$emit('site', event)
    },
    scienceMessage(msg) { // NOT YET IMPLEMENTED / MISSING IMPL, DOUBLE CHECK
      if (msg.type == 'auto_shutoff') {
        if (!msg.success) {
          this.autoShutdownEnabled = !this.autoShutdownEnabled
          alert('Toggling Auto Shutdown failed.')
        }
      }
    },
  },

  methods: {},
}
</script>

<style scoped></style>

<template>
  <div class="wrap p-2">
    <h3 class="mb-2 text-center">Arm Controls</h3>
    <div class="btn-group m-0 p-0" role="group" aria-label="Arm Modes">
      <button
        type="button"
        class="btn"
        :class="mode === 'disabled' ? 'btn-danger active' : 'btn-outline-danger'"
        @click="mode = 'disabled'"
      >
        Disabled
      </button>
      <button
        type="button"
        class="btn"
        :class="mode === 'throttle' ? 'btn-success active' : 'btn-outline-success'"
        @click="mode = 'throttle'"
      >
        Throttle
      </button>
    </div>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import Vuex from 'vuex'
const { mapActions } = Vuex

const UPDATE_HZ = 20

export default defineComponent({
  props: {
    currentSite: {
      type: Number,
      required: true,
    },
  },

  data() {
    return {
      mode: 'disabled',
      corer_position: 0,
      plunger_position: 0,
      sensor_height: 5.36,
      plunger_height: 5.5,
    }
  },

  created() {
    this.interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads()
      const gamepad = gamepads.find(
        gamepad => gamepad && gamepad.id.includes('Microsoft')
      )
      if (!gamepad) return

      this.$store.dispatch('websocket/sendMessage', {
        id: 'arm',
        message: {
          type: 'sa_controller',
          axes: gamepad.axes,
          buttons: gamepad.buttons.map(button => button.value),
        },
      })
      this.$store.dispatch('websocket/sendMessage', {
        id: 'arm',
        message: {
          type: 'sa_mode',
          mode: this.mode,
        },
      })
    }, 1000 / UPDATE_HZ)
  },

  beforeUnmount() {
    window.clearInterval(this.interval)
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),
  },
})
</script>

<style scoped>
.wrap {
  display: inline-flex;
  flex-direction: column;
  width: auto;
}
</style>

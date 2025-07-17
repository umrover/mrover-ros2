<template>
  <div class="wrap">
    <div class="d-flex justify-content-between align-items-center">
      <h2 class="m-0 me-5">Arm Controls</h2>
      <span
        class="px-2 py-2 rounded-2 text-black fw-semibold text-center"
        style="width: 130px; display: inline-block; font-family: monospace;"
        :class="controllerConnected ? 'bg-success' : 'bg-secondary'"
      >
        {{ controllerConnected ? 'Connected  ' : 'Disconnected' }}
      </span>
    </div>
    <div class="btn-group" role="group" aria-label="Arm mode selection">
      <button
        type="button"
        class="btn"
        :class="mode === 'disabled' ? 'btn-danger' : 'btn-outline-danger'"
        @click="mode = 'disabled'"
      >
        Disabled
      </button>
      <button
        type="button"
        class="btn"
        :class="mode === 'throttle' ? 'btn-success' : 'btn-outline-success'"
        @click="mode = 'throttle'"
      >
        Throttle
      </button>
      <button
        type="button"
        class="btn"
        :class="mode === 'ik-pos' ? 'btn-success' : 'btn-outline-success'"
        @click="mode = 'ik-pos'"
      >
        IK Position
      </button>
      <button
        type="button"
        class="btn"
        :class="mode === 'ik-vel' ? 'btn-success' : 'btn-outline-success'"
        @click="mode = 'ik-vel'"
      >
        IK Velocity
      </button>
    </div>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import Vuex from 'vuex'
const { mapActions, mapState } = Vuex

const UPDATE_HZ = 30

export default defineComponent({
  data() {
    return {
      mode: 'disabled',
      gamepadConnected: false,
    }
  },
  computed: {
    ...mapState('websocket', ['message']),
    controllerConnected(): boolean {
      return this.gamepadConnected
    },
  },
  mounted() {
    document.addEventListener('keydown', this.keyDown)
  },
  created() {
    this.interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads()
      const gamepad = gamepads.find(
        gamepad => gamepad && gamepad.id.includes('Microsoft'),
      )
      this.gamepadConnected = !!gamepad
      if (!gamepad) return

      this.$store.dispatch('websocket/sendMessage', {
        id: 'arm',
        message: {
          type: 'ra_controller',
          axes: gamepad.axes,
          buttons: gamepad.buttons.map(button => button.value),
        },
      })

      this.$store.dispatch('websocket/sendMessage', {
        id: 'arm',
        message: {
          type: 'ra_mode',
          mode: this.mode,
        },
      })
    }, 1000 / UPDATE_HZ)
  },
  beforeUnmount() {
    window.clearInterval(this.interval)
    document.removeEventListener('keydown', this.keyDown)
  },
  methods: {
    ...mapActions('websocket', ['sendMessage']),
    keyDown(event: { key: string }) {
      if (event.key === ' ') {
        this.mode = 'disabled'
      }
    },
  },
})
</script>

<style scoped>
.wrap {
  display: flex;
  flex-direction: column;
  align-items: center;
  width: 100%;
}
</style>

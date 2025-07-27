<template>
  <div class="d-flex flex-column align-items-center w-100">
    <div class="d-flex flex-column gap-2" style="width: 500px; max-width: 100%">
      <div class="d-flex justify-content-between align-items-center">
        <h3 class="m-0">Arm Controls</h3>
      </div>
      <div
        class="btn-group d-flex justify-content-between"
        role="group"
        aria-label="Arm mode selection"
      >
        <button
          type="button"
          class="btn flex-fill"
          :class="mode === 'disabled' ? 'btn-danger' : 'btn-outline-danger'"
          @click="mode = 'disabled'"
        >
          Disabled
        </button>
        <button
          type="button"
          class="btn flex-fill"
          :class="mode === 'throttle' ? 'btn-success' : 'btn-outline-success'"
          @click="mode = 'throttle'"
        >
          Throttle
        </button>
      </div>
    </div>
  </div>
</template>
<script lang="ts">
import { defineComponent } from 'vue'
import Vuex from 'vuex'
const { mapActions } = Vuex

const UPDATE_HZ = 30

export default defineComponent({
  data() {
    return {
      mode: 'disabled',
      // State to track which keys are currently pressed
      keysPressed: {
        w: false,
        a: false,
        s: false,
        d: false,
      },
      interval: 0,
    }
  },
  mounted() {
    // Add listeners for both keydown and keyup
    document.addEventListener('keydown', this.handleKeyDown)
    document.addEventListener('keyup', this.handleKeyUp)
  },
  created() {
    this.interval = window.setInterval(() => {
      const axes = [0, 0, 0, 0] // Default axes state
      const buttons: boolean[] = []
      axes[0] = (this.keysPressed.d ? 1 : 0) - (this.keysPressed.a ? 1 : 0)
      axes[1] = (this.keysPressed.s ? 1 : 0) - (this.keysPressed.w ? 1 : 0)

      this.$store.dispatch('websocket/sendMessage', {
        id: 'arm',
        message: {
          type: 'ra_controller',
          axes: axes,
          buttons: buttons,
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
    // Clean up listeners
    document.removeEventListener('keydown', this.handleKeyDown)
    document.removeEventListener('keyup', this.handleKeyUp)
  },
  methods: {
    ...mapActions('websocket', ['sendMessage']),
    handleKeyDown(event: KeyboardEvent) {
      const key = event.key.toLowerCase()
      if (key === ' ') {
        this.mode = 'disabled'
      }
      if (key in this.keysPressed) {
        this.keysPressed[key as keyof typeof this.keysPressed] = true
      }
    },
    handleKeyUp(event: KeyboardEvent) {
      const key = event.key.toLowerCase()
      if (key in this.keysPressed) {
        this.keysPressed[key as keyof typeof this.keysPressed] = false
      }
    },
  },
})
</script>

<template>
  <div class="d-flex flex-column align-items-center w-100">
    <div class="d-flex flex-column gap-2" style="width: 500px; max-width: 100%">
      <div class="d-flex justify-content-between align-items-center">
        <h3 class="m-0">Arm Controls</h3>
      </div>
      <h1>body</h1>
      <!-- TODO -->
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
    document.addEventListener('keydown', this.handleKeyDown)
    document.addEventListener('keyup', this.handleKeyUp)
  },
  created() {
    this.interval = window.setInterval(() => {
      const axes = [0, 0, 0, 0]
      const buttons: boolean[] = []
      axes[0] = (this.keysPressed.d ? 1 : 0) - (this.keysPressed.a ? 1 : 0)
      axes[1] = (this.keysPressed.s ? 1 : 0) - (this.keysPressed.w ? 1 : 0)

      // TODO

    }, 1000 / UPDATE_HZ)
  },
  beforeUnmount() {
    window.clearInterval(this.interval)
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

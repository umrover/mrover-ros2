<template>
  <div class="wrap">
    <h2>Arm Controls</h2>
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
const { mapActions, mapState } = Vuex;

const UPDATE_HZ = 30

export default defineComponent({
  components: {},
  data() {
    return {
      mode: 'disabled',
    }
  },

  computed: {
    ...mapState('websocket', ['message']),
  },

  mounted: function () {
    document.addEventListener('keydown', this.keyDown)
  },

  created: function () {
    this.interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads()
      // may need to check for Xbox rather than Microsoft
      const gamepad = gamepads.find(
        gamepad => gamepad && gamepad.id.includes('Microsoft'),
      )
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

  beforeUnmount: function () {
    window.clearInterval(this.interval)
    document.removeEventListener('keydown', this.keyDown)
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    keyDown: function (event: { key: string }) {
      // Use the space bar as an e-stop
      if (event.key == ' ') {
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
  justify-items: center;
  width: 100%;
  height: auto;
}

.wrap h2 h4 {
  margin: 0;
  font-size: 1.5em;
  font-weight: bold;
  text-align: center;
  width: 100%;
  padding: 5px 0;
}

.controls-flex {
  flex-wrap: wrap;
  display: flex;
  align-items: center;
  width: 100%;
  column-gap: 20px;
  padding-left: 10px;
  margin-bottom: 5px;
  margin-top: 5px;
}
</style>

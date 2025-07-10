<template>
  <div class="wrap">
    <h2>SA Arm Controls</h2>
    <div class="controls-flex">
      <h4>Mode</h4>
      <input
        v-model="mode"
        type="radio"
        class="btn-check"
        name="options-outlined"
        id="disabled"
        value="disabled"
        autocomplete="off"
        checked
      />
      <label class="btn btn-outline-danger" for="disabled">Disabled</label>
      <input
        v-model="mode"
        type="radio"
        class="btn-check"
        name="options-outlined"
        id="throttle"
        value="throttle"
        autocomplete="off"
      />
      <label class="btn btn-outline-success" for="throttle">Throttle</label>
    </div>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import Vuex from 'vuex'
const { mapActions, mapState } = Vuex

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

  created: function () {
    this.interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads()
      const gamepad = gamepads.find(
        gamepad => gamepad && gamepad.id.includes('Microsoft'),
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

  computed: {
    ...mapState('websocket', ['message']),
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),
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

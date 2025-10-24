<template>
  <div class="drive"></div>
</template>

<script lang='ts'>
import Vuex from 'vuex'
const { mapActions } = Vuex;

const UPDATE_HZ = 20

export default {
  methods: {
    ...mapActions('websocket', ['sendMessage'])
  },

  beforeUnmount: function() {
    window.clearInterval(this.interval)
  },

  created: function() {
    this.interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads()
      const gamepad = gamepads.find(gamepad => gamepad && gamepad.id.includes('Thrustmaster'))
      if (!gamepad) return

      const inverse_axes = gamepad.axes.map((value, index) => index === 1 ? -value : value)

      this.$store.dispatch('websocket/sendMessage', {
        id: 'drive',
        message: {
          type: 'joystick',
          // inverted controls, get rid of map after testing
          axes: inverse_axes,
          buttons: gamepad.buttons.map(button => button.value)
        },
      })
    }, 1000 / UPDATE_HZ)
  }
}
</script>

<style scoped>
.drive {
  display: none;
}
</style>

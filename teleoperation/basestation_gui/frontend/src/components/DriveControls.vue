<template>
  <div class="d-flex flex-column w-100 h-100">
    <div class="d-flex justify-content-between align-items-center w-100">
      <h4 class="component-header">Drive Controls</h4>
      <IndicatorDot :is-active="controllerConnected" />
    </div>
  </div>
</template>

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

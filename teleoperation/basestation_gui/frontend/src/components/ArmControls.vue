<template>
  <div class='wrap'>
    <h2>Arm Controls</h2>
    <div class='controls-flex'>
      <h4>Mode</h4>
        <input v-model='mode' type="radio" class="btn-check" name="options-outlined" id="disabled" value='disabled' autocomplete="off" checked>
        <label class="btn btn-outline-danger" for="disabled">Disabled</label>
        <input v-model='mode' type="radio" class="btn-check" name="options-outlined" id="throttle" value='throttle' autocomplete="off">
        <label class="btn btn-outline-success" for="throttle">Throttle</label>
        <input v-model='mode' type="radio" class="btn-check" name="options-outlined" id="ik-pos" value='ik-pos' autocomplete="off">
        <label class="btn btn-outline-success" for="ik-pos">IK Position</label>
        <input v-model='mode' type="radio" class="btn-check" name="options-outlined" id="ik-vel" value='ik-vel' autocomplete="off">
        <label class="btn btn-outline-success" for="ik-vel">IK Velocity</label>
    </div>
  </div>
</template>

<script lang='ts'>
import { defineComponent } from 'vue'
import { mapActions, mapState } from 'vuex'

const UPDATE_HZ = 20

export default defineComponent({
  components: {
  },
  data() {
    return {
      mode: 'disabled'
    }
  },

  computed: {
    ...mapState('websocket', ['message'])
  },

  mounted: function() {
    document.addEventListener('keydown', this.keyDown)
  },

  created: function() {
    this.interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads()
      // may need to check for Xbox rather than Microsoft
      const gamepad = gamepads.find(gamepad => gamepad && gamepad.id.includes('Microsoft'))
      if (!gamepad) return

      this.sendMessage({
        type: 'ra_controller',
        axes: gamepad.axes,
        buttons: gamepad.buttons.map(button => button.value)
      })

      this.sendMessage({
        type: 'ra_mode',
        mode: this.mode
      })
    }, 1000 / UPDATE_HZ)
  },

  beforeUnmount: function() {
    window.clearInterval(this.interval)
    document.removeEventListener('keydown', this.keyDown)
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    keyDown: function(event: { key: string }) {
      // Use the space bar as an e-stop
      if (event.key == ' ') {
        this.mode = 'disabled'
      }
    }
  }
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

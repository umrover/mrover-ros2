<template>
    <div>
        <p>Drive Controls</p>
        <div>
          <p>Left Motor Output: {{left}}</p>
          <p>Right Motor Output:  {{right}}</p>
        </div>
  
    </div>
  </template>
  
  <script>
  import { mapActions } from 'vuex'
  import { mapState } from 'vuex'
  
  const UPDATE_HZ = 20
  
  export default {
    data() {
      return {
        left: 0,
        right: 0,
      }
    },
  
    watch: {
      message(msg) {
        if (msg.type == 'wheel_cmd') {
          this.left = msg.left;
          this.right = msg.right;
        }
      }
    },
  
    methods: {
      ...mapActions('websocket', ['sendMessage'])
    },
  
    computed: {
      ...mapState('websocket', ['message'])
    },
  
    beforeUnmount: function() {
      window.clearInterval(this.interval)
    },
  
    created: function() {
      this.interval = window.setInterval(() => {
        /* 
          To test this code either plug in the Thrustmaster and use this code:
        */
  
        // const gamepads = navigator.getGamepads()
        // const gamepad = gamepads.find(gamepad => gamepad && gamepad.id.includes('Thrustmaster'))
        // if (!gamepad) return
  
        // this.sendMessage({
        //   type: 'joystick',
        //   axes: gamepad.axes,
        //   buttons: gamepad.buttons.map(button => button.value)
        // })
  
        /* 
          OR, test the code with hardcoded values:
        */
        const axes = [0, 0.75, 0.1, 0.5, 0, 0] // index 1 affects forward/back, 2 affects twist/turn, 3 affects throttle
        const buttons = Array(16).fill(0) // 16 buttons available to map to. None of them are "pressed" currently
  
        this.sendMessage({
          type: 'joystick',
          axes: axes,
          buttons: buttons
        })
        
      }, 1000 / UPDATE_HZ)
    }
  }
  </script>
  
  <style scoped>
  </style>
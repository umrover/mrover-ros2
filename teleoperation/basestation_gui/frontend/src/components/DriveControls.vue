
<script lang='ts' setup>
import { onMounted, onBeforeUnmount } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'

const websocketStore = useWebsocketStore()

let interval: number | undefined = undefined

const UPDATE_HZ = 20

onMounted(() => {
  interval = window.setInterval(() => {
    const gamepads = navigator.getGamepads()
    const gamepad = gamepads.find(gamepad => gamepad && gamepad.id.includes('Thrustmaster'))
    if (!gamepad) return

    const inverse_axes = gamepad.axes.map((value, index) => index === 1 ? -value : value)

    websocketStore.sendMessage('drive', {
      type: 'joystick',
      // inverted controls, get rid of map after testing
      axes: inverse_axes,
      buttons: gamepad.buttons.map(button => button.value)
    })
  }, 1000 / UPDATE_HZ)
})

onBeforeUnmount(() => {
  window.clearInterval(interval)
})
</script>



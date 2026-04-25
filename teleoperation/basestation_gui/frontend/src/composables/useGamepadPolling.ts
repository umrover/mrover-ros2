import { ref, onMounted, onBeforeUnmount, type Ref } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'

export interface UseGamepadPollingOptions {
  controllerIdFilter: string
<<<<<<< HEAD
  topic: string
  messageType: string
=======
  topic?: string
  messageType?: string
>>>>>>> origin/main
  hz?: number
  transformAxes?: (axes: number[]) => number[]
}

export interface UseGamepadPollingReturn {
  connected: Ref<boolean>
  axes: Ref<number[]>
  buttons: Ref<number[]>
<<<<<<< HEAD
=======
  vibrationActuator: Ref<GamepadHapticActuator | undefined>
>>>>>>> origin/main
}

export function useGamepadPolling(options: UseGamepadPollingOptions): UseGamepadPollingReturn {
  const { controllerIdFilter, topic, messageType, hz = 15, transformAxes } = options
<<<<<<< HEAD
  const websocketStore = useWebsocketStore()
=======
  const websocketStore = topic && messageType ? useWebsocketStore() : null
>>>>>>> origin/main

  const connected = ref(false)
  const axes = ref<number[]>([0, 0, 0, 0])
  const buttons = ref<number[]>(new Array(17).fill(0))
<<<<<<< HEAD
=======
  const vibrationActuator = ref<GamepadHapticActuator>()
>>>>>>> origin/main

  let interval: number | undefined

  onMounted(() => {
    interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads()
      const gamepad = gamepads.find(gp => gp && gp.id.includes(controllerIdFilter))
      connected.value = !!gamepad
      if (!gamepad) return

<<<<<<< HEAD
=======
      vibrationActuator.value = gamepad.vibrationActuator
>>>>>>> origin/main
      const rawAxes = Array.from(gamepad.axes)
      const mappedButtons = gamepad.buttons.map(b => b.value)

      axes.value = transformAxes ? transformAxes(rawAxes) : rawAxes
      buttons.value = mappedButtons

<<<<<<< HEAD
      const sendAxes = transformAxes ? transformAxes(rawAxes) : rawAxes
      websocketStore.sendMessage(topic, {
        type: messageType,
        axes: sendAxes,
        buttons: mappedButtons,
      })
=======
      if (websocketStore && topic && messageType) {
        websocketStore.sendMessage(topic, {
          type: messageType,
          axes: transformAxes ? transformAxes(rawAxes) : rawAxes,
          buttons: mappedButtons,
        })
      }
>>>>>>> origin/main
    }, 1000 / hz)
  })

  onBeforeUnmount(() => {
    if (interval !== undefined) {
      window.clearInterval(interval)
    }
  })

<<<<<<< HEAD
  return { connected, axes, buttons }
=======
  return { connected, axes, buttons, vibrationActuator }
>>>>>>> origin/main
}

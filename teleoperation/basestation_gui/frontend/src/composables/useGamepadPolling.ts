import { ref, onMounted, onBeforeUnmount, type Ref } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'

export interface UseGamepadPollingOptions {
  controllerIdFilter: string
  topic: string
  messageType: string
  hz?: number
  transformAxes?: (axes: number[]) => number[]
}

export interface UseGamepadPollingReturn {
  connected: Ref<boolean>
  axes: Ref<number[]>
  buttons: Ref<number[]>
}

export function useGamepadPolling(options: UseGamepadPollingOptions): UseGamepadPollingReturn {
  const { controllerIdFilter, topic, messageType, hz = 15, transformAxes } = options
  const websocketStore = useWebsocketStore()

  const connected = ref(false)
  const axes = ref<number[]>([0, 0, 0, 0])
  const buttons = ref<number[]>(new Array(17).fill(0))

  let interval: number | undefined

  onMounted(() => {
    interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads()
      const gamepad = gamepads.find(gp => gp && gp.id.includes(controllerIdFilter))
      connected.value = !!gamepad
      if (!gamepad) return

      const rawAxes = Array.from(gamepad.axes)
      const mappedButtons = gamepad.buttons.map(b => b.value)

      axes.value = transformAxes ? transformAxes(rawAxes) : rawAxes
      buttons.value = mappedButtons

      const sendAxes = transformAxes ? transformAxes(rawAxes) : rawAxes
      websocketStore.sendMessage(topic, {
        type: messageType,
        axes: sendAxes,
        buttons: mappedButtons,
      })
    }, 1000 / hz)
  })

  onBeforeUnmount(() => {
    if (interval !== undefined) {
      window.clearInterval(interval)
    }
  })

  return { connected, axes, buttons }
}

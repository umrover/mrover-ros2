import { ref, onMounted, onBeforeUnmount, type Ref } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'

export interface UseGamepadOptions {
  controllerIdFilter: string
  hz?: number
  transformAxes?: (axes: number[]) => number[]
  ws?: {
    topic: string
    messageType: string
  }
}

export interface UseGamepadReturn {
  connected: Ref<boolean>
  axes: Ref<number[]>
  buttons: Ref<number[]>
}

export function useGamepad(options: UseGamepadOptions): UseGamepadReturn {
  const { controllerIdFilter, hz = 15, transformAxes, ws } = options

  const connected = ref(false)
  const axes = ref<number[]>([0, 0, 0, 0])
  const buttons = ref<number[]>(new Array(17).fill(0))

  const websocketStore = ws ? useWebsocketStore() : null

  let interval: number | undefined

  onMounted(() => {
    interval = window.setInterval(() => {
      const gamepads = navigator.getGamepads()
      const gamepad = gamepads.find(gp => gp && gp.id.includes(controllerIdFilter))
      connected.value = !!gamepad
      if (!gamepad) return

      const rawAxes = Array.from(gamepad.axes)
      const mappedButtons = gamepad.buttons.map(b => b.value)
      const finalAxes = transformAxes ? transformAxes(rawAxes) : rawAxes

      axes.value = finalAxes
      buttons.value = mappedButtons

      if (websocketStore && ws) {
        websocketStore.sendMessage(ws.topic, {
          type: ws.messageType,
          axes: finalAxes,
          buttons: mappedButtons,
        })
      }
    }, 1000 / hz)
  })

  onBeforeUnmount(() => {
    if (interval !== undefined) {
      window.clearInterval(interval)
    }
  })

  return { connected, axes, buttons }
}

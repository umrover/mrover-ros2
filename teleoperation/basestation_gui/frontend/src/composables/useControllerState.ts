import { ref, onUnmounted } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import type { ControllerStateMessage } from '@/types/websocket'

const STALE_TIMEOUT_MS = 1000

export function useStaleTimer() {
  const stale = ref(true)
  let timer: ReturnType<typeof setTimeout> | null = null

  function reset() {
    stale.value = false
    if (timer) clearTimeout(timer)
    timer = setTimeout(() => { stale.value = true }, STALE_TIMEOUT_MS)
  }

  onUnmounted(() => { if (timer) clearTimeout(timer) })

  return { stale, reset }
}

export interface ControllerState {
  names: string[]
  states: string[]
  errors: string[]
  positions: number[]
  velocities: number[]
  currents: number[]
  limitsHit: number[]
}

function emptyControllerState(): ControllerState {
  return { names: [], states: [], errors: [], positions: [], velocities: [], currents: [], limitsHit: [] }
}

function applyMessage(target: ControllerState, msg: ControllerStateMessage) {
  target.names = msg.names
  target.states = msg.states
  target.errors = msg.errors
  target.positions = msg.positions
  target.velocities = msg.velocities
  target.currents = msg.currents
  target.limitsHit = msg.limits_hit
}

interface ControllerMessageOptions {
  topic: string
  messageType: string
}

export function useControllerMessage(options: ControllerMessageOptions) {
  const { onMessage } = useWebsocketStore()
  const { stale, reset } = useStaleTimer()
  const data = ref<ControllerState>(emptyControllerState())

  onMessage<ControllerStateMessage>(options.topic, options.messageType, (msg) => {
    reset()
    applyMessage(data.value, msg)
  })

  return { stale, data }
}

export function formatState(v: string | undefined): string {
  return v || '---'
}

export function formatNumber(v: unknown): string {
  if (typeof v === 'number' && Number.isFinite(v)) return v.toFixed(2)
  return '---'
}

export function formatLimit(v: number | undefined): string {
  if (v === undefined) return '---'
  return String(v)
}

export function formatError(v: string | undefined): string {
  if (v === undefined) return '---'
  if (v === '') return 'None'
  return v
}

function hasError(v: string | undefined): boolean {
  return !!v && v !== 'None'
}

export function stateRowClass(state: string | undefined, error?: string | undefined): string {
  if (!state) return 'row-no-data'
  if (hasError(error)) return 'bg-cmd-danger-subtle'
  if (state === 'ARMED') return 'bg-cmd-success-subtle'
  return ''
}

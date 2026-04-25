import { ref, onUnmounted } from 'vue'
<<<<<<< HEAD
=======
import { useWebsocketStore } from '@/stores/websocket'
import type { ControllerStateMessage } from '@/types/websocket'
>>>>>>> origin/main

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

<<<<<<< HEAD
export function formatState(v: string | undefined): string {
  return v || '---'
}

export function formatNumber(v: unknown): string {
  if (typeof v === 'number' && Number.isFinite(v)) return v.toFixed(2)
  return '---'
}

=======
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

export const MOTEUS_MAP: Record<string, string> = {
  'Motor Stopped': 'STOP',
  'Motor Fault': 'FAULT',
  'Motor Enabling': 'ENBL',
  'Motor Calibrating': 'CAL',
  'Motor Calibration Complete': 'DONE',
  'Motor Pwm': 'PWM',
  'Voltage Operating Mode': 'VOLT',
  'Voltage FOC Operating Mode': 'FOC',
  'Voltage DQ Operating Mode': 'DQ',
  'Current Operating Mode': 'CURR',
  'Position Operating Mode': 'POS',
  'Position Timeout': 'TMOUT',
  'Zero Velocity': 'ZERO',
  'Motor Stay Within': 'STAY',
  'Measure Ind': 'MEAS',
  'Motor Brake': 'BRAKE',
}

export const BMC_MAP: Record<string, string> = {
  'Running': 'RUN',
  'Fault': 'FLT',
}

export const SERVO_MAP: Record<string, string> = {
  'Active': 'ACT',
  'HardwareFailure': 'HW_FLT',
  'Success': 'OK',
  'FailedToOpenPort': 'PORT_ERR',
  'FailedToSetBaud': 'BAUD_ERR',
  'CommPortBusy': 'BUSY',
  'CommTxFail': 'TX_FAIL',
  'CommRxFail': 'RX_FAIL',
  'CommTxError': 'TX_ERR',
  'CommRxWaiting': 'WAIT',
  'CommRxTimeout': 'RX_TMOUT',
  'CommRxCorrupt': 'CORR',
  'CommNotAvailable': 'N/A',
}

export const SIM_MAP: Record<string, string> = {
  'Armed': 'ARMED',
}

export const STATE_MAP: Record<string, string> = {
  ...MOTEUS_MAP,
  ...BMC_MAP,
  ...SERVO_MAP,
  ...SIM_MAP,
}

export function formatState(v: string | undefined): string {
  if (!v) return '---'
  return STATE_MAP[v] || v
}

export { formatNumber } from '@/utils/formatNumber'

>>>>>>> origin/main
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
<<<<<<< HEAD
  return !!v && v !== 'None'
=======
  if (!v) return false
  const normalized = v.toLowerCase()
  return normalized !== 'none' && normalized !== 'no error' && normalized !== 'success' && normalized !== ''
>>>>>>> origin/main
}

export function stateRowClass(state: string | undefined, error?: string | undefined): string {
  if (!state) return 'row-no-data'
<<<<<<< HEAD
  if (hasError(error)) return 'bg-cmd-danger-subtle'
  if (state === 'ARMED') return 'bg-cmd-success-subtle'
=======
  const isFaultState = state.toLowerCase().includes('fault') || state.toLowerCase().includes('failure')
  if (isFaultState || hasError(error)) return 'bg-danger-subtle'
>>>>>>> origin/main
  return ''
}

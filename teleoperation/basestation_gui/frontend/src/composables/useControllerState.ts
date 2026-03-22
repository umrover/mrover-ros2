import { ref, onUnmounted } from 'vue'

export { formatNumber, formatState, formatLimit, formatError } from '@/utils/formatNumber'

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

function hasError(v: string | undefined): boolean {
  return !!v && v !== 'None'
}

export function stateRowClass(state: string | undefined, error?: string | undefined): string {
  if (!state) return 'row-no-data'
  if (hasError(error)) return 'bg-cmd-danger-subtle'
  if (state === 'ARMED') return 'bg-cmd-success-subtle'
  return ''
}

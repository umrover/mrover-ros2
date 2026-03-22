const FIGURE_DASH = '\u2012'

export function formatNumber(value: unknown, intDigits = 3, fracDigits = 2, signed = false): string {
  if (typeof value !== 'number' || !Number.isFinite(value)) {
    const signSlot = signed ? `<span class="fmt-dim">${FIGURE_DASH}</span>` : ''
    const decimal = fracDigits > 0 ? '.' + FIGURE_DASH.repeat(fracDigits) : ''
    return `${signSlot}<span class="fmt-dim">${FIGURE_DASH.repeat(intDigits)}${decimal}</span>`
  }
  const abs = Math.abs(value)
  const fixed = abs.toFixed(fracDigits)
  const intPart = fixed.split('.')[0]
  const padCount = Math.max(0, intDigits - intPart.length)
  const pad = padCount > 0 ? `<span class="fmt-dim">${'0'.repeat(padCount)}</span>` : ''
  let sign = ''
  if (signed) {
    sign = value < 0 ? '-' : `<span class="fmt-dim">+</span>`
  }
  return `${sign}${pad}${fixed}`
}

export function formatState(v: string | undefined): string {
  return v || '---'
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

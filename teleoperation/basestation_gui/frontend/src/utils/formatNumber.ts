const FIGURE_DASH = '\u2012'

export interface FormattedNumber {
  sign: string
  signDim: boolean
  pad: string
  num: string
  noData: boolean
}

export function formatNumber(value: unknown, intDigits = 3, fracDigits = 2): FormattedNumber {
  if (typeof value !== 'number' || !Number.isFinite(value)) {
    const decimal = fracDigits > 0 ? '.' + FIGURE_DASH.repeat(fracDigits) : ''
    const placeholder = FIGURE_DASH.repeat(intDigits) + decimal
    return { sign: FIGURE_DASH, signDim: true, pad: '', num: placeholder, noData: true }
  }
  const abs = Math.abs(value)
  const fixed = abs.toFixed(fracDigits)
  const intPart = fixed.split('.')[0]
  const padCount = Math.max(0, intDigits - intPart.length)
  return {
    sign: value < 0 ? '-' : '+',
    signDim: value >= 0,
    pad: '0'.repeat(padCount),
    num: fixed,
    noData: false,
  }
}

export function formatValue(v: number | undefined, intDigits = 3, fracDigits = 2): FormattedNumber {
  return formatNumber(v, intDigits, fracDigits)
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

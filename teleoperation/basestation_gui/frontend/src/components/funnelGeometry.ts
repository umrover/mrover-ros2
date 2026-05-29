export const DEG_TO_RAD = Math.PI / 180
export const RAD_TO_DEG = 180 / Math.PI

export interface Site {
  label: string
  radians: number
}

export const SITES: Site[] = [
  { label: 'Sample',   radians: 3.1415 },
  { label: 'Buret A',  radians: 2.0071 },
  { label: 'Griess A', radians: 1.1693 },
  { label: 'Trash',    radians: 0.0 },
  { label: 'Buret B',  radians: 5.1138 },
  { label: 'Griess B', radians: 4.2586 },
]

export const CX = 100
export const CY = 100

const INNER_R = 40
const OUTER_R = 100
const LABEL_R = 80
const LABEL_R_FLIP = 90
const TARGET_R = INNER_R + 11
const TARGET_R_FLIP = INNER_R + 18
const GAP_HALF = 3
const SLOT_SIZE = 360 / SITES.length

function polarToXY(angleDeg: number, r: number): { x: number; y: number } {
  const rad = (angleDeg - 90) * DEG_TO_RAD
  return { x: CX + r * Math.cos(rad), y: CY + r * Math.sin(rad) }
}

function gapEdgePoint(gapDeg: number, segCenterDeg: number, r: number): { x: number; y: number } {
  const gRad = (gapDeg - 90) * DEG_TO_RAD
  const offsetSign = Math.sign(Math.sin((segCenterDeg - gapDeg) * DEG_TO_RAD))
  const t = Math.sqrt(r * r - GAP_HALF * GAP_HALF)
  return {
    x: CX + GAP_HALF * offsetSign * -Math.sin(gRad) + t * Math.cos(gRad),
    y: CY + GAP_HALF * offsetSign * Math.cos(gRad) + t * Math.sin(gRad),
  }
}

function arcPath(gap1: number, gap2: number, center: number, r1: number, r2: number): string {
  const p1 = gapEdgePoint(gap1, center, r1)
  const p2 = gapEdgePoint(gap2, center, r1)
  const p3 = gapEdgePoint(gap2, center, r2)
  const p4 = gapEdgePoint(gap1, center, r2)
  const f = (n: number) => n.toFixed(2)
  return [
    `M${f(p1.x)},${f(p1.y)}`,
    `A${r1},${r1} 0 0 1 ${f(p2.x)},${f(p2.y)}`,
    `L${f(p3.x)},${f(p3.y)}`,
    `A${r2},${r2} 0 0 0 ${f(p4.x)},${f(p4.y)}`,
    'Z',
  ].join(' ')
}

function labelArcPath(gap1: number, gap2: number, r: number, flip: boolean): string {
  const from = flip ? gap2 : gap1
  const to = flip ? gap1 : gap2
  const p1 = polarToXY(from, r)
  const p2 = polarToXY(to, r)
  const f = (n: number) => n.toFixed(2)
  return `M${f(p1.x)},${f(p1.y)} A${r},${r} 0 0 ${flip ? 0 : 1} ${f(p2.x)},${f(p2.y)}`
}

export interface Segment {
  index: number
  label: string
  path: string
  labelArc: string
  targetArc: string
}

export const segments: Segment[] = SITES.map((site, i) => {
  const centerDeg = i * SLOT_SIZE
  const gap1 = centerDeg - SLOT_SIZE / 2
  const gap2 = centerDeg + SLOT_SIZE / 2
  const flip = centerDeg > 90 && centerDeg < 270
  return {
    index: i,
    label: site.label,
    path: arcPath(gap1, gap2, centerDeg, INNER_R, OUTER_R),
    labelArc: labelArcPath(gap1, gap2, flip ? LABEL_R_FLIP : LABEL_R, flip),
    targetArc: labelArcPath(gap1, gap2, flip ? TARGET_R_FLIP : TARGET_R, flip),
  }
})

export function siteTargetDeg(siteIndex: number, offsetDeg: number): number {
  const raw = SITES[siteIndex].radians * RAD_TO_DEG + offsetDeg
  return Math.round(((raw % 360) + 360) % 360)
}

export function closestSiteIndex(positionRad: number, offsetDeg: number, maxDiffRad = 0.2): number {
  const offsetRad = offsetDeg * DEG_TO_RAD
  let closestIdx = -1
  let minDiff = Infinity
  for (let i = 0; i < SITES.length; i++) {
    const target = SITES[i].radians + offsetRad
    const diff = Math.abs(((target - positionRad + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI)
    if (diff < minDiff) {
      minDiff = diff
      closestIdx = i
    }
  }
  return minDiff < maxDiffRad ? closestIdx : -1
}

export function siteTargetRad(siteIndex: number, offsetDeg: number): number {
  const raw = SITES[siteIndex].radians + offsetDeg * DEG_TO_RAD
  return ((raw % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI)
}

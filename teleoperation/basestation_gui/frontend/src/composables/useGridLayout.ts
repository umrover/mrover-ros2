import { ref, watch, onMounted, onUnmounted, nextTick } from 'vue'
import { useGridLayoutStore } from '@/stores/gridLayout'
import { storeToRefs } from 'pinia'

export interface LayoutItem {
  x: number
  y: number
  w: number
  h: number
  i: string
}

const GRID_ROWS = 12
const MARGIN = 10

export function useGridLayout(layoutKey: string, defaultLayout: LayoutItem[]) {
  const wrapperRef = ref<HTMLElement | null>(null)
  const rowHeight = ref(30)

  const calculateRowHeight = () => {
    if (!wrapperRef.value) return
    const availableHeight = wrapperRef.value.clientHeight
    const totalMargin = MARGIN * (GRID_ROWS + 1)
    rowHeight.value = Math.floor((availableHeight - totalMargin) / GRID_ROWS)
  }

  const loadLayout = (): LayoutItem[] => {
    const saved = localStorage.getItem(layoutKey)
    if (saved) {
      try {
        return JSON.parse(saved)
      } catch {
        return defaultLayout
      }
    }
    return defaultLayout
  }

  const layout = ref<LayoutItem[]>(loadLayout())

  const saveLayout = () => {
    localStorage.setItem(layoutKey, JSON.stringify(layout.value))
  }

  const gridLayoutStore = useGridLayoutStore()
  const { locked, resetCounter } = storeToRefs(gridLayoutStore)

  watch(resetCounter, () => {
    layout.value = JSON.parse(JSON.stringify(defaultLayout))
    saveLayout()
  })

  onMounted(() => {
    nextTick(() => {
      calculateRowHeight()
    })
    window.addEventListener('resize', calculateRowHeight)
  })

  onUnmounted(() => {
    window.removeEventListener('resize', calculateRowHeight)
  })

  return {
    wrapperRef,
    rowHeight,
    layout,
    locked,
    saveLayout,
    GRID_ROWS,
    MARGIN,
  }
}

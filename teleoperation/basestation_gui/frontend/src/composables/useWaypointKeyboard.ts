import { ref, computed, watch } from 'vue'
import { useAutonomyStore } from '@/stores/autonomy'
import type { AutonWaypoint } from '@/types/waypoints'

export type Column = 'store' | 'execution'
export type KeyboardMode = 'NORMAL' | 'VISUAL'

function rangeToSet(anchor: number, cursor: number): Set<number> {
  const start = Math.min(anchor, cursor)
  const end = Math.max(anchor, cursor)
  const indices = new Set<number>()
  for (let i = start; i <= end; i++) indices.add(i)
  return indices
}

export function useWaypointKeyboard() {
  const autonomyStore = useAutonomyStore()

  const mode = ref<KeyboardMode>('NORMAL')
  const focusedColumn = ref<Column>('store')
  const storeIndex = ref(0)
  const executionIndex = ref(0)
  const showCheatSheet = ref(false)

  let onEnterCallback: (() => void) | null = null

  const storeVisualAnchor = ref(-1)
  const executionVisualAnchor = ref(-1)

  let pendingKey = ''
  let pendingTimeout: ReturnType<typeof setTimeout> | null = null

  const storeSelectedIndices = computed<Set<number>>(() => {
    if (mode.value !== 'VISUAL' || storeVisualAnchor.value < 0) return new Set()
    return rangeToSet(storeVisualAnchor.value, storeIndex.value)
  })

  const executionSelectedIndices = computed<Set<number>>(() => {
    if (mode.value !== 'VISUAL' || executionVisualAnchor.value < 0) return new Set()
    return rangeToSet(executionVisualAnchor.value, executionIndex.value)
  })

  function getColumnList(column?: Column): AutonWaypoint[] {
    const col = column ?? focusedColumn.value
    return col === 'store' ? autonomyStore.store : autonomyStore.execution
  }

  function getColumnIndexRef(column?: Column) {
    const col = column ?? focusedColumn.value
    return col === 'store' ? storeIndex : executionIndex
  }

  function clampIndex(column?: Column) {
    const col = column ?? focusedColumn.value
    const indexRef = getColumnIndexRef(col)
    const list = getColumnList(col)
    if (list.length === 0) {
      indexRef.value = 0
    } else if (indexRef.value >= list.length) {
      indexRef.value = list.length - 1
    }
  }

  watch(() => autonomyStore.store.length, () => clampIndex('store'))
  watch(() => autonomyStore.execution.length, () => clampIndex('execution'))

  function moveDown() {
    const list = getColumnList()
    const idx = getColumnIndexRef()
    if (idx.value < list.length - 1) idx.value++
  }

  function moveUp() {
    const idx = getColumnIndexRef()
    if (idx.value > 0) idx.value--
  }

  function jumpTop() {
    getColumnIndexRef().value = 0
  }

  function jumpBottom() {
    const list = getColumnList()
    getColumnIndexRef().value = Math.max(0, list.length - 1)
  }

  function moveLeft() {
    if (mode.value === 'VISUAL') return
    if (focusedColumn.value === 'execution') {
      focusedColumn.value = 'store'
      clampIndex()
    }
  }

  function moveRight() {
    if (mode.value === 'VISUAL') return
    if (focusedColumn.value === 'store') {
      focusedColumn.value = 'execution'
      clampIndex()
    }
  }

  function getVisualAnchorRef(column?: Column) {
    const col = column ?? focusedColumn.value
    return col === 'store' ? storeVisualAnchor : executionVisualAnchor
  }

  function getSelectedIndices(column?: Column) {
    const col = column ?? focusedColumn.value
    return col === 'store' ? storeSelectedIndices : executionSelectedIndices
  }

  function enterVisualMode() {
    const list = getColumnList()
    if (list.length === 0) return
    if (focusedColumn.value === 'execution' && autonomyStore.isNavigating) return
    mode.value = 'VISUAL'
    getVisualAnchorRef().value = getColumnIndexRef().value
  }

  function exitVisualMode() {
    mode.value = 'NORMAL'
    storeVisualAnchor.value = -1
    executionVisualAnchor.value = -1
  }

  function setOnEnter(cb: (() => void) | null) {
    onEnterCallback = cb
  }

  async function performStage() {
    if (focusedColumn.value !== 'store') return

    if (mode.value === 'VISUAL' && storeVisualAnchor.value >= 0) {
      const indices = storeSelectedIndices.value
      if (indices.size === 0) return
      const start = Math.min(...indices)
      const end = Math.max(...indices)
      const waypoints = autonomyStore.store.slice(start, end + 1)
      await autonomyStore.addManyToExecution(waypoints)
      exitVisualMode()
      return
    }

    const wp = autonomyStore.store[storeIndex.value]
    if (wp) {
      await autonomyStore.addToExecution(wp)
    }
  }

  async function performDelete() {
    if (mode.value === 'VISUAL') {
      const selected = getSelectedIndices()
      const indices = Array.from(selected.value)
      if (indices.length === 0) return

      if (focusedColumn.value === 'store') {
        await autonomyStore.removeMultipleFromStore(indices)
      } else {
        const waypoints = indices
          .map(i => autonomyStore.execution[i])
          .filter((wp): wp is AutonWaypoint => wp != null)
        for (const wp of waypoints.reverse()) {
          await autonomyStore.removeFromExecution(wp)
        }
      }
      exitVisualMode()
      clampIndex()
      return
    }

    const list = getColumnList()
    const idx = getColumnIndexRef()
    if (list.length === 0) return

    if (focusedColumn.value === 'store') {
      await autonomyStore.removeFromStore(idx.value)
    } else {
      const wp = list[idx.value]
      if (wp) await autonomyStore.removeFromExecution(wp)
    }
    clampIndex()
  }

  function reorderUp() {
    if (mode.value !== 'NORMAL') return
    if (focusedColumn.value === 'store') {
      if (storeIndex.value <= 0) return
      autonomyStore.reorderStore(storeIndex.value, storeIndex.value - 1)
      storeIndex.value--
    }
  }

  function reorderDown() {
    if (mode.value !== 'NORMAL') return
    if (focusedColumn.value === 'store') {
      if (storeIndex.value >= autonomyStore.store.length - 1) return
      autonomyStore.reorderStore(storeIndex.value, storeIndex.value + 1)
      storeIndex.value++
    }
  }

  const scrollKey = computed(() =>
    `${focusedColumn.value}:${getColumnIndexRef().value}`
  )

  function clearPending() {
    pendingKey = ''
    if (pendingTimeout) {
      clearTimeout(pendingTimeout)
      pendingTimeout = null
    }
  }

  function handleKeydown(e: KeyboardEvent) {
    if (e.key === '?') {
      e.preventDefault()
      showCheatSheet.value = !showCheatSheet.value
      return
    }

    if (e.key === 'Escape') {
      if (mode.value === 'VISUAL') {
        e.preventDefault()
        exitVisualMode()
        return
      }
      if (showCheatSheet.value) {
        showCheatSheet.value = false
        e.preventDefault()
      }
      return
    }

    if (pendingKey) {
      const seq = pendingKey + e.key
      clearPending()

      if (seq === 'gg') {
        e.preventDefault()
        jumpTop()
        return
      }
      if (seq === 'dd') {
        e.preventDefault()
        performDelete()
        return
      }
    }

    if (e.key === 'g' && !e.shiftKey) {
      pendingKey = 'g'
      pendingTimeout = setTimeout(clearPending, 500)
      e.preventDefault()
      return
    }
    if (e.key === 'd' && !e.shiftKey) {
      if (mode.value === 'VISUAL') {
        e.preventDefault()
        performDelete()
        return
      }
      pendingKey = 'd'
      pendingTimeout = setTimeout(clearPending, 500)
      e.preventDefault()
      return
    }

    switch (e.key) {
      case 'j':
      case 'ArrowDown':
        e.preventDefault()
        moveDown()
        break
      case 'k':
      case 'ArrowUp':
        e.preventDefault()
        moveUp()
        break
      case 'h':
      case 'ArrowLeft':
        e.preventDefault()
        moveLeft()
        break
      case 'l':
      case 'ArrowRight':
        e.preventDefault()
        moveRight()
        break
      case 'G':
        e.preventDefault()
        jumpBottom()
        break
      case 'Home':
        e.preventDefault()
        jumpTop()
        break
      case 'End':
        e.preventDefault()
        jumpBottom()
        break
      case 'J':
        e.preventDefault()
        reorderDown()
        break
      case 'K':
        e.preventDefault()
        reorderUp()
        break
      case 'Enter':
      case 'e':
        e.preventDefault()
        if (onEnterCallback) onEnterCallback()
        break
      case 'v':
        e.preventDefault()
        if (mode.value === 'VISUAL') {
          exitVisualMode()
        } else {
          enterVisualMode()
        }
        break
      case 's':
        e.preventDefault()
        performStage()
        break
    }
  }

  return {
    mode,
    focusedColumn,
    storeIndex,
    executionIndex,
    showCheatSheet,
    scrollKey,
    storeSelectedIndices,
    executionSelectedIndices,
    handleKeydown,
    setOnEnter,
    exitVisualMode,
  }
}

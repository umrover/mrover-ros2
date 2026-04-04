import { ref, computed, watch } from 'vue'
import { useAutonomyStore } from '@/stores/autonomy'
import type { AutonWaypoint } from '@/types/waypoints'

export type Column = 'store' | 'execution'
export type KeyboardMode = 'NORMAL' | 'INSERT' | 'VISUAL'

export function useWaypointKeyboard() {
  const autonomyStore = useAutonomyStore()

  const mode = ref<KeyboardMode>('NORMAL')
  const focusedColumn = ref<Column>('store')
  const storeIndex = ref(0)
  const executionIndex = ref(0)
  const showCheatSheet = ref(false)
  const editingStoreIndex = ref(-1)

  const visualAnchor = ref(-1)

  let pendingKey = ''
  let pendingTimeout: ReturnType<typeof setTimeout> | null = null

  const selectedRange = computed<{ start: number; end: number } | null>(() => {
    if (mode.value !== 'VISUAL' || visualAnchor.value < 0) return null
    const a = visualAnchor.value
    const b = storeIndex.value
    return { start: Math.min(a, b), end: Math.max(a, b) }
  })

  const selectedIndices = computed<Set<number>>(() => {
    const range = selectedRange.value
    if (!range) return new Set()
    const indices = new Set<number>()
    for (let i = range.start; i <= range.end; i++) {
      indices.add(i)
    }
    return indices
  })

  function getColumnList(column?: Column): AutonWaypoint[] {
    const col = column ?? focusedColumn.value
    switch (col) {
      case 'store': return autonomyStore.store
      case 'execution': return autonomyStore.execution
    }
  }

  function getColumnIndexRef(column?: Column) {
    const col = column ?? focusedColumn.value
    switch (col) {
      case 'store': return storeIndex
      case 'execution': return executionIndex
    }
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

  function enterVisualMode() {
    if (focusedColumn.value !== 'store') return
    if (autonomyStore.store.length === 0) return
    mode.value = 'VISUAL'
    visualAnchor.value = storeIndex.value
  }

  function exitVisualMode() {
    mode.value = 'NORMAL'
    visualAnchor.value = -1
  }

  function enterInsertMode() {
    if (focusedColumn.value !== 'store') return
    if (mode.value === 'VISUAL') return
    const list = getColumnList()
    if (list.length === 0) return
    mode.value = 'INSERT'
    editingStoreIndex.value = storeIndex.value
  }

  function exitInsertMode() {
    mode.value = 'NORMAL'
    editingStoreIndex.value = -1
  }

  async function performStage() {
    if (focusedColumn.value !== 'store') return

    if (mode.value === 'VISUAL') {
      const range = selectedRange.value
      if (!range) return
      const waypoints = autonomyStore.store.slice(range.start, range.end + 1)
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
    if (mode.value === 'VISUAL' && focusedColumn.value === 'store') {
      const range = selectedRange.value
      if (!range) return
      const indices = Array.from(selectedIndices.value)
      await autonomyStore.removeMultipleFromStore(indices)
      exitVisualMode()
      clampIndex()
      return
    }

    const list = getColumnList()
    const idx = getColumnIndexRef()
    if (list.length === 0) return

    if (focusedColumn.value === 'store') {
      await autonomyStore.removeFromStore(idx.value)
    } else if (focusedColumn.value === 'execution') {
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
    if (mode.value === 'INSERT') {
      if (e.key === 'Escape') {
        e.preventDefault()
        exitInsertMode()
      }
      return
    }

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
        e.preventDefault()
        enterInsertMode()
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
    editingStoreIndex,
    showCheatSheet,
    scrollKey,
    visualAnchor,
    selectedRange,
    selectedIndices,
    handleKeydown,
    exitInsertMode,
    exitVisualMode,
  }
}

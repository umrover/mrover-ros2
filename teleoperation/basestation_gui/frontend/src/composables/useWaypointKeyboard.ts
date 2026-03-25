import { ref, computed, watch } from 'vue'
import { useAutonomyStore } from '@/stores/autonomy'
import type { AutonWaypoint } from '@/types/waypoints'

export type Column = 'store' | 'staging' | 'execution'
export type KeyboardMode = 'NORMAL' | 'INSERT'

export function useWaypointKeyboard() {
  const autonomyStore = useAutonomyStore()

  const mode = ref<KeyboardMode>('NORMAL')
  const focusedColumn = ref<Column>('store')
  const storeIndex = ref(0)
  const stagingIndex = ref(0)
  const executionIndex = ref(0)
  const showCheatSheet = ref(false)
  const editingStoreIndex = ref(-1)

  let pendingKey = ''
  let pendingTimeout: ReturnType<typeof setTimeout> | null = null

  function getColumnList(column?: Column): AutonWaypoint[] {
    const col = column ?? focusedColumn.value
    switch (col) {
      case 'store': return autonomyStore.store
      case 'staging': return autonomyStore.staging
      case 'execution': return autonomyStore.execution
    }
  }

  function getColumnIndexRef(column?: Column) {
    const col = column ?? focusedColumn.value
    switch (col) {
      case 'store': return storeIndex
      case 'staging': return stagingIndex
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
  watch(() => autonomyStore.staging.length, () => clampIndex('staging'))
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
    const columns: Column[] = ['store', 'staging', 'execution']
    const i = columns.indexOf(focusedColumn.value)
    if (i > 0) {
      focusedColumn.value = columns[i - 1]
      clampIndex()
    }
  }

  function moveRight() {
    const columns: Column[] = ['store', 'staging', 'execution']
    const i = columns.indexOf(focusedColumn.value)
    if (i < columns.length - 1) {
      focusedColumn.value = columns[i + 1]
      clampIndex()
    }
  }

  function enterInsertMode() {
    if (focusedColumn.value !== 'store') return
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
    const list = getColumnList()
    const idx = getColumnIndexRef()
    const wp = list[idx.value]
    if (!wp) return

    if (focusedColumn.value === 'store') {
      await autonomyStore.addToStaging(wp)
    } else if (focusedColumn.value === 'staging') {
      await autonomyStore.stageToExecution(wp)
    }
    clampIndex()
  }

  async function performUnstage() {
    const list = getColumnList()
    const idx = getColumnIndexRef()
    const wp = list[idx.value]
    if (!wp) return

    if (focusedColumn.value === 'execution') {
      await autonomyStore.unstageOne(wp)
      clampIndex()
    }
  }

  async function performDelete() {
    const list = getColumnList()
    const idx = getColumnIndexRef()
    if (list.length === 0) return

    if (focusedColumn.value === 'store') {
      await autonomyStore.removeFromStore(idx.value)
    } else if (focusedColumn.value === 'staging') {
      const wp = list[idx.value]
      if (wp) await autonomyStore.removeFromStaging(wp)
    } else if (focusedColumn.value === 'execution') {
      const wp = list[idx.value]
      if (wp) await autonomyStore.removeFromExecution(wp)
    }
    clampIndex()
  }

  function reorderUp() {
    if (focusedColumn.value !== 'staging') return
    if (stagingIndex.value <= 0) return
    const next = [...autonomyStore.staging]
    const i = stagingIndex.value
    ;[next[i - 1], next[i]] = [next[i], next[i - 1]]
    autonomyStore.staging = next
    stagingIndex.value--
    autonomyStore.saveStaging()
  }

  function reorderDown() {
    if (focusedColumn.value !== 'staging') return
    if (stagingIndex.value >= autonomyStore.staging.length - 1) return
    const next = [...autonomyStore.staging]
    const i = stagingIndex.value
    ;[next[i], next[i + 1]] = [next[i + 1], next[i]]
    autonomyStore.staging = next
    stagingIndex.value++
    autonomyStore.saveStaging()
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
      case 's':
        e.preventDefault()
        performStage()
        break
      case 'u':
        e.preventDefault()
        performUnstage()
        break
    }
  }

  return {
    mode,
    focusedColumn,
    storeIndex,
    stagingIndex,
    executionIndex,
    editingStoreIndex,
    showCheatSheet,
    scrollKey,
    handleKeydown,
    exitInsertMode,
  }
}

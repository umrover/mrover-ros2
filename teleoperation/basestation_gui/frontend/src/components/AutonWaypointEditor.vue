<template>
  <div
    class="wrapper flex m-0 p-0 h-full w-full gap-2 relative outline-none"
    tabindex="0"
    @keydown="handleKeydown"
    @focus="editorFocused = true"
    @blur="handleBlur"
    ref="editorRef"
  >
    <div class="editor-column" :class="{ 'column-focused': editorFocused && keyboard.focusedColumn.value === 'store' }">
      <div class="waypoint-header p-2 mb-2 flex justify-between items-center border-b">
        <div class="flex items-center gap-2">
          <h4 class="component-header">Waypoint Store</h4>
          <button
            class="btn btn-sm btn-icon-sm"
            :class="focusLocked ? 'btn-primary' : 'btn-secondary'"
            title="GET VIMMING"
            @click="toggleFocusLock"
          >
            <i class="bi bi-lock-fill"></i>
          </button>
        </div>
        <div class="flex gap-2 items-center">
          <button
            class="btn btn-danger btn-sm btn-icon-sm"
            data-testid="pw-reset-waypoints-btn"
            @click="resetModal?.open()"
            title="Reset waypoints"
          >
            <i class="bi bi-arrow-clockwise"></i>
          </button>
          <button class="btn btn-sm btn-success" data-testid="pw-add-from-map" @click="addModal?.open()">
            Add from Map
          </button>
        </div>
      </div>
      <div class="waypoint-wrapper p-2 rounded grow overflow-auto relative" data-testid="pw-waypoint-store-list">
        <div v-if="autonomyStore.store.length === 0" class="course-empty-state">
          <i class="bi bi-geo-alt"></i>
          <span>No waypoints in store</span>
        </div>
        <WaypointStore
          v-for="(waypoint, index) in autonomyStore.store"
          :key="waypoint.db_id || index"
          :waypoint="waypoint"
          :index="index"
          :highlighted="editorFocused && keyboard.focusedColumn.value === 'store' && keyboard.storeIndex.value === index"
          :editing="keyboard.editingStoreIndex.value === index"
          @add="autonomyStore.addToStaging"
          @delete="autonomyStore.removeFromStore"
          @update="handleStoreUpdate"
          @cancel-edit="handleCancelEdit"
          @save-edit="handleInlineEdit"
        />
      </div>
    </div>

    <div class="editor-column" :class="{ 'column-focused': editorFocused && keyboard.focusedColumn.value === 'staging' }">
      <div class="waypoint-header p-2 mb-2 flex justify-between items-center border-b">
        <h4 class="component-header">Staging</h4>
        <div class="flex gap-1">
          <button
            class="btn btn-sm btn-danger"
            :disabled="autonomyStore.staging.length === 0"
            @click="autonomyStore.clearStaging()"
          >
            Clear
          </button>
          <button
            class="btn btn-sm btn-success"
            :disabled="autonomyStore.staging.length === 0"
            @click="autonomyStore.stageAllToExecution()"
          >
            Execute All
          </button>
        </div>
      </div>
      <VueDraggable
        v-model="autonomyStore.staging"
        handle=".drag-handle"
        ghost-class="drag-ghost"
        class="waypoint-wrapper p-2 rounded grow overflow-auto relative"
        @end="autonomyStore.saveStaging()"
      >
        <div v-if="autonomyStore.staging.length === 0" class="course-empty-state">
          <i class="bi bi-signpost-split"></i>
          <span>No waypoints staged</span>
        </div>
        <WaypointItem
          v-for="(element, index) in autonomyStore.staging"
          :key="element.db_id ?? index"
          :waypoint="element"
          :highlighted="editorFocused && keyboard.focusedColumn.value === 'staging' && keyboard.stagingIndex.value === index"
          @stage="autonomyStore.stageToExecution(element)"
          @delete="autonomyStore.removeFromStaging(element)"
        />
      </VueDraggable>
    </div>

    <div class="editor-column" :class="{ 'column-focused': editorFocused && keyboard.focusedColumn.value === 'execution' }">
      <div class="waypoint-header p-2 mb-2 flex justify-between items-center border-b">
        <h4 class="component-header">Execution</h4>
        <div class="flex gap-1">
          <button
            class="btn btn-sm btn-warning"
            :disabled="autonomyStore.execution.length === 0 || autonomyStore.isNavigating"
            data-testid="pw-unstage-execution"
            @click="autonomyStore.unstageExecution()"
          >
            Return All
          </button>
          <button
            class="btn btn-sm btn-danger"
            :disabled="autonomyStore.execution.length === 0 || autonomyStore.isNavigating"
            @click="autonomyStore.clearExecution()"
          >
            Clear
          </button>
          <button
            class="btn btn-sm btn-success"
            :disabled="autonomyStore.staging.length === 0 || autonomyStore.isNavigating"
            data-testid="pw-stage-next"
            @click="autonomyStore.stageNext()"
          >
            Execute Next
          </button>
        </div>
      </div>
      <AutonExecutionPanel
        :highlighted-index="editorFocused && keyboard.focusedColumn.value === 'execution' ? keyboard.executionIndex.value : -1"
      />
    </div>

    <div v-if="keyboard.showCheatSheet.value" class="kbd-cheatsheet" @click="keyboard.showCheatSheet.value = false">
      <div class="kbd-cheatsheet-content" @click.stop>
        <div class="flex justify-between items-center mb-3">
          <h5 class="font-bold">Keyboard Shortcuts</h5>
          <button class="btn-close" @click="keyboard.showCheatSheet.value = false"><i class="bi bi-x-lg"></i></button>
        </div>
        <div class="grid grid-cols-2 gap-4">
          <div>
            <h6 class="font-semibold mb-1">Navigation</h6>
            <div class="kbd-row"><kbd>h</kbd><kbd>l</kbd> or <kbd>&larr;</kbd><kbd>&rarr;</kbd> Switch column</div>
            <div class="kbd-row"><kbd>j</kbd><kbd>k</kbd> or <kbd>&darr;</kbd><kbd>&uarr;</kbd> Move highlight</div>
            <div class="kbd-row"><kbd>g</kbd><kbd>g</kbd> or <kbd>Home</kbd> Jump to top</div>
            <div class="kbd-row"><kbd>G</kbd> or <kbd>End</kbd> Jump to bottom</div>
          </div>
          <div>
            <h6 class="font-semibold mb-1">Actions</h6>
            <div class="kbd-row"><kbd>Enter</kbd> Edit waypoint (store only)</div>
            <div class="kbd-row"><kbd>s</kbd> Stage forward</div>
            <div class="kbd-row"><kbd>u</kbd> Unstage (execution only)</div>
            <div class="kbd-row"><kbd>d</kbd><kbd>d</kbd> Delete</div>
            <div class="kbd-row"><kbd>J</kbd><kbd>K</kbd> Reorder (staging only)</div>
          </div>
          <div>
            <h6 class="font-semibold mb-1">Insert Mode</h6>
            <div class="kbd-row"><kbd>Tab</kbd> / <kbd>Shift+Tab</kbd> Cycle fields</div>
            <div class="kbd-row"><kbd>Enter</kbd> Save</div>
            <div class="kbd-row"><kbd>Escape</kbd> Cancel</div>
          </div>
          <div>
            <h6 class="font-semibold mb-1">Other</h6>
            <div class="kbd-row"><kbd>?</kbd> Toggle this sheet</div>
            <div class="kbd-row"><kbd>Escape</kbd> Close overlay</div>
          </div>
        </div>
      </div>
    </div>
  </div>

  <AutonAddWaypointModal ref="addModal" />

  <ConfirmModal
    ref="resetModal"
    modal-id="modalReset"
    title="Reset Waypoints"
    confirm-text="Reset"
    @confirm="autonomyStore.resetAll()"
  >
    <p>This will clear all user-added waypoints, staging, and execution.</p>
    <p class="text-muted mb-0">Default waypoints will be preserved.</p>
  </ConfirmModal>
</template>

<script lang="ts" setup>
import { ref, nextTick, watch, onMounted } from 'vue'
import WaypointItem from './AutonWaypointItem.vue'
import WaypointStore from './AutonWaypointStore.vue'
import AutonAddWaypointModal from './AutonAddWaypointModal.vue'
import AutonExecutionPanel from './AutonExecutionPanel.vue'
import ConfirmModal from './ConfirmModal.vue'
import { VueDraggable } from 'vue-draggable-plus'
import type { AutonWaypoint } from '@/types/waypoints'
import { useAutonomyStore } from '@/stores/autonomy'
import { useWaypointKeyboard } from '@/composables/useWaypointKeyboard'

const autonomyStore = useAutonomyStore()
const keyboard = useWaypointKeyboard()

const editorRef = ref<HTMLElement | null>(null)
const editorFocused = ref(false)
const LS_FOCUS_LOCK = 'autonEditor.focusLocked'
const focusLocked = ref(localStorage.getItem(LS_FOCUS_LOCK) === 'true')
const addModal = ref<InstanceType<typeof AutonAddWaypointModal> | null>(null)
const resetModal = ref<InstanceType<typeof ConfirmModal> | null>(null)

onMounted(() => {
  autonomyStore.fetchAll()
  if (focusLocked.value) {
    editorRef.value?.focus()
  }
})

function scrollToHighlighted() {
  nextTick(() => {
    const el = editorRef.value?.querySelector('.kbd-highlighted')
    if (el) {
      el.scrollIntoView({ block: 'nearest', behavior: 'smooth' })
    }
  })
}

watch(() => keyboard.scrollKey.value, () => {
  if (!editorFocused.value) return
  scrollToHighlighted()
})

watch(() => keyboard.editingStoreIndex.value, (idx) => {
  if (idx >= 0) scrollToHighlighted()
})

function toggleFocusLock() {
  focusLocked.value = !focusLocked.value
  localStorage.setItem(LS_FOCUS_LOCK, String(focusLocked.value))
  if (focusLocked.value) {
    editorRef.value?.focus()
  } else {
    editorRef.value?.blur()
  }
}

function handleKeydown(e: KeyboardEvent) {
  if (focusLocked.value && e.key === 'Tab' && keyboard.mode.value === 'NORMAL') {
    e.preventDefault()
    return
  }
  keyboard.handleKeydown(e)
}

function handleStoreUpdate(waypoint: AutonWaypoint, index: number) {
  const existing = autonomyStore.store[index]
  if (!existing || existing.db_id == null) return

  const fields: Partial<AutonWaypoint> = {}
  if (waypoint.name !== existing.name) fields.name = waypoint.name
  if (waypoint.lat !== existing.lat) fields.lat = waypoint.lat
  if (waypoint.lon !== existing.lon) fields.lon = waypoint.lon
  if (Object.keys(fields).length > 0) {
    autonomyStore.updateStore(existing.db_id, fields)
  }
}

function handleBlur(e: FocusEvent) {
  const wrapper = editorRef.value
  if (wrapper && e.relatedTarget instanceof Node && wrapper.contains(e.relatedTarget)) {
    return
  }
  if (focusLocked.value) {
    nextTick(() => wrapper?.focus())
    return
  }
  editorFocused.value = false
}

function refocusEditor() {
  editorRef.value?.focus()
}

function handleInlineEdit(waypoint: AutonWaypoint, index: number) {
  handleStoreUpdate(waypoint, index)
  keyboard.exitInsertMode()
  refocusEditor()
}

function handleCancelEdit() {
  keyboard.exitInsertMode()
  refocusEditor()
}
</script>

<style scoped>
.editor-column {
  display: flex;
  flex: 1;
  flex-direction: column;
  min-width: 0;
  border: 2px solid transparent;
  border-radius: var(--radius-md);
  transition: border-color 0.15s;
}

.column-focused {
  border-color: var(--color-primary);
}

.waypoint-wrapper {
  scrollbar-gutter: stable;
  background-color: var(--view-bg);
}
</style>

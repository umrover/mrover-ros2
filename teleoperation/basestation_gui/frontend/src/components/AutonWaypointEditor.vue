<template>
  <div
    class="wrapper flex m-0 p-0 h-full w-full gap-2 relative outline-none"
    :tabindex="vimEnabled ? 0 : -1"
    @keydown="handleKeydown"
    @focus="editorFocused = true"
    @blur="handleBlur"
    ref="editorRef"
  >
    <div class="editor-column" :class="{ 'column-focused': vimEnabled && editorFocused && keyboard.focusedColumn.value === 'store' }">
      <div class="waypoint-header p-2 mb-2 flex justify-between items-center border-b">
        <div class="flex items-center gap-2">
          <h4 class="component-header">Waypoint Store</h4>
          <button
            class="vim-toggle"
            :class="vimEnabled ? 'vim-toggle--on' : 'vim-toggle--off'"
            title="Toggle Vim keybindings"
            @click="toggleVim"
          >
            <img src="/vim-logo.svg" alt="Vim" class="vim-logo" />
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
      <VueDraggable
        v-model="autonomyStore.store"
        :group="{ name: 'waypoints', pull: 'clone', put: false }"
        :filter="'.btn, input, select'"
        :prevent-on-filter="false"
        ghost-class="drag-ghost"
        class="waypoint-wrapper p-2 rounded grow overflow-auto relative"
        data-testid="pw-waypoint-store-list"
        @end="handleStoreDragEnd"
      >
        <div v-if="autonomyStore.store.length === 0" class="course-empty-state">
          <i class="bi bi-geo-alt"></i>
          <span>No waypoints in store</span>
        </div>
        <WaypointStore
          v-for="(waypoint, index) in autonomyStore.store"
          :key="waypoint.db_id || index"
          :waypoint="waypoint"
          :index="index"
          :highlighted="vimEnabled && editorFocused && keyboard.focusedColumn.value === 'store' && keyboard.storeIndex.value === index"
          :visual-selected="vimEnabled && keyboard.storeSelectedIndices.value.has(index)"
          :editing="vimEnabled && keyboard.editingStoreIndex.value === index"
          @add="autonomyStore.addToExecution"
          @delete="autonomyStore.removeFromStore"
          @update="handleStoreUpdate"
          @cancel-edit="handleCancelEdit"
          @save-edit="handleInlineEdit"
        />
      </VueDraggable>
      <div v-if="vimEnabled && keyboard.storeSelectedIndices.value.size > 0" class="visual-mode-bar">
        -- VISUAL -- ({{ keyboard.storeSelectedIndices.value.size }} selected)
      </div>
    </div>

    <div class="editor-column" :class="{ 'column-focused': vimEnabled && editorFocused && keyboard.focusedColumn.value === 'execution' }">
      <div class="waypoint-header p-2 mb-2 flex justify-between items-center border-b">
        <h4 class="component-header">Execution</h4>
        <div class="flex gap-1">
          <button
            class="btn btn-sm btn-danger"
            :disabled="autonomyStore.execution.length === 0 || autonomyStore.isNavigating"
            @click="autonomyStore.clearExecution()"
          >
            Clear
          </button>
        </div>
      </div>
      <VueDraggable
        v-model="autonomyStore.execution"
        :group="{ name: 'waypoints', pull: false, put: !autonomyStore.isNavigating }"
        ghost-class="drag-ghost"
        class="waypoint-wrapper p-2 rounded grow overflow-auto relative"
        :disabled="autonomyStore.isNavigating"
        @add="handleExecutionDragAdd"
        @end="handleExecutionDragEnd"
      >
        <div v-if="autonomyStore.execution.length === 0" class="course-empty-state">
          <i class="bi bi-cursor"></i>
          <span>No active waypoints</span>
        </div>
        <div
          v-for="(wp, index) in autonomyStore.execution"
          :key="wp.db_id ?? index"
          class="list-item"
          :class="{
            'kbd-highlighted': vimEnabled && editorFocused && keyboard.focusedColumn.value === 'execution' && keyboard.executionIndex.value === index,
            'kbd-visual-selected': vimEnabled && keyboard.executionSelectedIndices.value.has(index),
          }"
        >
          <div class="flex justify-between items-center mb-1">
            <h5 class="list-item-title">{{ wp.name }}</h5>
            <span v-if="wp.tag_id != null" class="data-label">#{{ wp.tag_id }}</span>
          </div>
          <div class="flex justify-between items-center">
            <small class="text-muted">{{ wp.lat.toFixed(6) }}N, {{ wp.lon.toFixed(6) }}W</small>
            <button
              class="btn btn-sm btn-danger btn-icon"
              :disabled="autonomyStore.isNavigating"
              @click="autonomyStore.removeFromExecution(wp)"
            >
              <i class="bi bi-trash-fill" />
            </button>
          </div>
        </div>
      </VueDraggable>
      <div v-if="vimEnabled && keyboard.executionSelectedIndices.value.size > 0" class="visual-mode-bar">
        -- VISUAL -- ({{ keyboard.executionSelectedIndices.value.size }} selected)
      </div>
    </div>

    <div v-if="vimEnabled && keyboard.showCheatSheet.value" class="kbd-cheatsheet" @click="keyboard.showCheatSheet.value = false">
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
            <div class="kbd-row"><kbd>Enter</kbd> Edit waypoint (store)</div>
            <div class="kbd-row"><kbd>s</kbd> Stage to execution</div>
            <div class="kbd-row"><kbd>d</kbd><kbd>d</kbd> Delete</div>
            <div class="kbd-row"><kbd>J</kbd><kbd>K</kbd> Reorder (store)</div>
          </div>
          <div>
            <h6 class="font-semibold mb-1">Visual Mode</h6>
            <div class="kbd-row"><kbd>v</kbd> Enter/exit visual mode</div>
            <div class="kbd-row"><kbd>j</kbd><kbd>k</kbd> Extend selection</div>
            <div class="kbd-row"><kbd>s</kbd> Stage all selected</div>
            <div class="kbd-row"><kbd>d</kbd><kbd>d</kbd> Delete all selected</div>
            <div class="kbd-row"><kbd>Escape</kbd> Cancel selection</div>
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
    <p>This will clear all user-added waypoints and execution.</p>
    <p class="text-muted mb-0">Default waypoints will be preserved.</p>
  </ConfirmModal>
</template>

<script lang="ts" setup>
import { ref, nextTick, watch, onMounted } from 'vue'
import WaypointStore from './AutonWaypointStore.vue'
import AutonAddWaypointModal from './AutonAddWaypointModal.vue'
import ConfirmModal from './ConfirmModal.vue'
import { VueDraggable } from 'vue-draggable-plus'
import type { AutonWaypoint } from '@/types/waypoints'
import { useAutonomyStore } from '@/stores/autonomy'
import { useWaypointKeyboard } from '@/composables/useWaypointKeyboard'

const autonomyStore = useAutonomyStore()
const keyboard = useWaypointKeyboard()

const editorRef = ref<HTMLElement | null>(null)
const editorFocused = ref(false)
const addModal = ref<InstanceType<typeof AutonAddWaypointModal> | null>(null)
const resetModal = ref<InstanceType<typeof ConfirmModal> | null>(null)

const LS_VIM = 'autonEditor.vimEnabled'
const vimEnabled = ref(localStorage.getItem(LS_VIM) === 'true')

onMounted(() => {
  autonomyStore.fetchAll()
  if (vimEnabled.value) {
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
  if (!vimEnabled.value || !editorFocused.value) return
  scrollToHighlighted()
})

watch(() => keyboard.editingStoreIndex.value, (idx) => {
  if (vimEnabled.value && idx >= 0) scrollToHighlighted()
})

function toggleVim() {
  vimEnabled.value = !vimEnabled.value
  localStorage.setItem(LS_VIM, String(vimEnabled.value))
  if (vimEnabled.value) {
    editorRef.value?.focus()
  } else {
    keyboard.exitVisualMode()
    keyboard.exitInsertMode()
    editorRef.value?.blur()
    editorFocused.value = false
  }
}

function handleKeydown(e: KeyboardEvent) {
  if (!vimEnabled.value) return
  if (e.key === 'Tab' && keyboard.mode.value === 'NORMAL') {
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
  if (!vimEnabled.value) {
    editorFocused.value = false
    return
  }
  const wrapper = editorRef.value
  if (wrapper && e.relatedTarget instanceof Node && wrapper.contains(e.relatedTarget)) {
    return
  }
  nextTick(() => wrapper?.focus())
}

function refocusEditor() {
  if (vimEnabled.value) {
    editorRef.value?.focus()
  }
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

function handleStoreDragEnd() {
  // Store reorder happened via VueDraggable v-model binding
}

function handleExecutionDragAdd(evt: { newIndex: number }) {
  const wp = autonomyStore.execution[evt.newIndex]
  if (!wp) return
  const isDuplicate = autonomyStore.execution.some(
    (existing, i) => i !== evt.newIndex && existing.db_id === wp.db_id
  )
  if (isDuplicate) {
    const next = [...autonomyStore.execution]
    next.splice(evt.newIndex, 1)
    autonomyStore.execution = next
    return
  }
  autonomyStore.saveExecution()
}

function handleExecutionDragEnd() {
  autonomyStore.saveExecution()
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
  border-color: var(--status-ok);
}

.waypoint-wrapper {
  scrollbar-gutter: stable;
  background-color: var(--view-bg);
}

.visual-mode-bar {
  padding: 0.25rem 0.75rem;
  font-size: 0.75rem;
  font-weight: 600;
  color: var(--status-ok);
  text-align: center;
  letter-spacing: 0.05em;
  background-color: var(--view-bg);
  border-top: 2px solid var(--panel-border);
}

.list-item-title {
  font-size: 0.875rem;
  font-weight: 600;
}

.data-label {
  font-size: 0.6875rem;
  color: var(--text-muted);
}

.btn-icon {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  width: 1.75rem;
  height: 1.75rem;
  padding: 0;
  font-size: 0.75rem;
}

.vim-toggle {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  width: 2rem;
  height: 2rem;
  padding: 3px;
  border-radius: var(--radius-sm);
  cursor: pointer;
  transition: all 0.15s;
}

.vim-toggle--on {
  border: 2px solid var(--status-ok);
  background-color: rgba(var(--status-ok-rgb), 0.15);
  box-shadow: 0 0 6px rgba(var(--status-ok-rgb), 0.3);
}

.vim-toggle--off {
  border: 2px solid var(--panel-border);
  background-color: transparent;
  opacity: 0.35;
}

.vim-toggle--off:hover {
  opacity: 0.7;
  border-color: var(--text-muted);
}

.vim-logo {
  width: 100%;
  height: 100%;
  object-fit: contain;
}

</style>

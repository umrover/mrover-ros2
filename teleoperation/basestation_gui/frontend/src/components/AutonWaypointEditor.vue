<template>
  <div
    class="wrapper flex m-0 p-0 h-full w-full gap-2 relative outline-none"
    :tabindex="vim.vimEnabled.value ? 0 : -1"
    @keydown="vim.handleKeydown"
    @focus="vim.editorFocused.value = true"
    @blur="vim.handleBlur"
    ref="editorRef"
  >
    <div class="editor-column" :class="{ 'column-focused': vim.vimEnabled.value && vim.editorFocused.value && vim.keyboard.focusedColumn.value === 'store' }">
      <div class="waypoint-header p-2 mb-2 flex justify-between items-center border-b">
        <div class="flex items-center gap-2">
          <h4 class="component-header">Waypoint Items</h4>
          <button
            class="vim-toggle"
            :class="vim.vimEnabled.value ? 'vim-toggle--on' : 'vim-toggle--off'"
            title="Toggle Vim keybindings"
            @click="vim.toggleVim"
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
      >
        <div v-if="autonomyStore.store.length === 0" class="course-empty-state">
          <i class="bi bi-geo-alt"></i>
          <span>No waypoints in store</span>
        </div>
        <WaypointItem
          v-for="(waypoint, index) in autonomyStore.store"
          :key="waypoint.db_id || index"
          :ref="(el: any) => { if (el) storeItemRefs[index] = el }"
          :waypoint="waypoint"
          :index="index"
          :highlighted="vim.vimEnabled.value && vim.editorFocused.value && vim.keyboard.focusedColumn.value === 'store' && vim.keyboard.storeIndex.value === index"
          :visual-selected="vim.vimEnabled.value && vim.keyboard.storeSelectedIndices.value.has(index)"
          :on-colon="() => commandBar?.open()"
          @add="autonomyStore.addToExecution"
          @delete="autonomyStore.removeFromStore"
          @update="handleStoreUpdate"
        />
      </VueDraggable>
    </div>

    <div class="editor-column" :class="{ 'column-focused': vim.vimEnabled.value && vim.editorFocused.value && vim.keyboard.focusedColumn.value === 'execution' }">
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
            'kbd-highlighted': vim.vimEnabled.value && vim.editorFocused.value && vim.keyboard.focusedColumn.value === 'execution' && vim.keyboard.executionIndex.value === index,
            'kbd-visual-selected': vim.vimEnabled.value && vim.keyboard.executionSelectedIndices.value.has(index),
          }"
        >
          <div class="flex justify-between items-center mb-1">
            <h5 class="list-item-title">{{ wp.name }}</h5>
            <span v-if="wp.type === 1 && wp.tag_id != null" class="data-label">#{{ wp.tag_id }}</span>
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
    </div>

    <div v-if="vim.vimEnabled.value && vim.keyboard.showCheatSheet.value" class="kbd-cheatsheet" @click="vim.keyboard.showCheatSheet.value = false">
      <div class="kbd-cheatsheet-content" @click.stop>
        <div class="flex justify-between items-center mb-3">
          <h5 class="font-bold">Keyboard Shortcuts</h5>
          <button class="btn-close" @click="vim.keyboard.showCheatSheet.value = false"><i class="bi bi-x-lg"></i></button>
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
            <div class="kbd-row"><kbd>Enter</kbd> / <kbd>e</kbd> Edit waypoint (store)</div>
            <div class="kbd-row"><kbd>s</kbd> Stage to execution</div>
            <div class="kbd-row"><kbd>d</kbd><kbd>d</kbd> Delete</div>
            <div class="kbd-row"><kbd>J</kbd><kbd>K</kbd> Reorder (store)</div>
          </div>
          <div>
            <h6 class="font-semibold mb-1">Visual Mode</h6>
            <div class="kbd-row"><kbd>v</kbd> Enter/exit visual mode</div>
            <div class="kbd-row"><kbd>j</kbd><kbd>k</kbd> Extend selection</div>
            <div class="kbd-row"><kbd>s</kbd> Stage all selected</div>
            <div class="kbd-row"><kbd>d</kbd> Delete all selected</div>
            <div class="kbd-row"><kbd>Escape</kbd> Cancel selection</div>
          </div>
          <div>
            <h6 class="font-semibold mb-1">Command Mode</h6>
            <div class="kbd-row"><kbd>:</kbd> Open command bar</div>
            <div class="kbd-row"><kbd>:e</kbd> Edit waypoint</div>
            <div class="kbd-row"><kbd>:w</kbd> Save edit</div>
            <div class="kbd-row"><kbd>:q</kbd> Close edit</div>
            <div class="kbd-row"><kbd>:wq</kbd> / <kbd>:x</kbd> Save and close</div>
            <div class="kbd-row"><kbd>?</kbd> Toggle this sheet</div>
          </div>
        </div>
      </div>
    </div>
  </div>

  <Teleport to="body">
    <div v-if="vim.vimEnabled.value && vim.visualSelectedCount.value > 0" class="visual-mode-bar">
      -- VISUAL -- ({{ vim.visualSelectedCount.value }} selected)
    </div>
  </Teleport>

  <VimBar v-if="vim.vimEnabled.value" ref="commandBar" @execute="vim.handleVimCommand" @close="vim.restoreFocus" />

  <AutonWaypointModal ref="addModal" />

  <ConfirmModal
    ref="resetModal"
    modal-id="modalReset"
    title="Reset Waypoints"
    confirm-text="Reset"
    @confirm="autonomyStore.resetAll()"
  >
    <p>THIS WILL DROP AND RECREATE DB TABLES</p>
    <p>THIS WILL REMOVE ALL USER-ADDED WAYPOINTS</p>
  </ConfirmModal>
</template>

<script lang="ts" setup>
import { ref, onMounted } from 'vue'
import WaypointItem from './AutonWaypointItem.vue'
import AutonWaypointModal from './AutonWaypointModal.vue'
import ConfirmModal from './ConfirmModal.vue'
import VimBar from './VimBar.vue'
import { VueDraggable } from 'vue-draggable-plus'
import type { AutonWaypoint } from '@/types/waypoints'
import { useAutonomyStore } from '@/stores/autonomy'
import { useVim } from '@/composables/useVim'

const autonomyStore = useAutonomyStore()

const editorRef = ref<HTMLElement | null>(null)
const addModal = ref<InstanceType<typeof AutonWaypointModal> | null>(null)
const resetModal = ref<InstanceType<typeof ConfirmModal> | null>(null)
const storeItemRefs: Record<number, InstanceType<typeof WaypointItem>> = {}
const commandBar = ref<InstanceType<typeof VimBar> | null>(null)

const vim = useVim({ editorRef, storeItemRefs, commandBar })

onMounted(() => {
  autonomyStore.fetchAll()
})

function handleStoreUpdate(waypoint: AutonWaypoint, index: number) {
  const existing = autonomyStore.store[index]
  if (!existing || existing.db_id == null) return

  const fields: Partial<AutonWaypoint> = {}
  if (waypoint.name !== existing.name) fields.name = waypoint.name
  if (waypoint.type !== existing.type) fields.type = waypoint.type
  if (waypoint.tag_id !== existing.tag_id) fields.tag_id = waypoint.tag_id
  if (waypoint.lat !== existing.lat) fields.lat = waypoint.lat
  if (waypoint.lon !== existing.lon) fields.lon = waypoint.lon
  if (Object.keys(fields).length > 0) {
    autonomyStore.updateStore(existing.db_id, fields)
  }
}

function handleExecutionDragAdd(evt: { newIndex?: number }) {
  if (evt.newIndex == null) return
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

<style>
.visual-mode-bar {
  position: fixed;
  bottom: 0;
  left: 0;
  right: 0;
  padding: 0.25rem 0.75rem;
  font-size: 0.75rem;
  font-weight: 600;
  color: #fff;
  text-align: center;
  letter-spacing: 0.05em;
  background-color: var(--status-ok);
  border-top: 2px solid var(--status-ok);
  z-index: 1000;
}
</style>

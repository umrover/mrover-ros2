<template>
  <div class="wrapper flex m-0 p-0 h-full w-full gap-2">
    <div class="editor-column">
      <div class="waypoint-header p-2 mb-2 flex justify-between items-center border-b">
        <h4 class="component-header">Waypoint Store</h4>
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
          @add="autonomyStore.addToStaging"
          @delete="autonomyStore.removeFromStore"
          @update="handleStoreUpdate"
        />
      </div>
    </div>

    <div class="editor-column">
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
          @stage="autonomyStore.stageToExecution(element)"
          @delete="autonomyStore.removeFromStaging(element)"
        />
      </VueDraggable>
    </div>

    <div class="editor-column">
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
      <AutonExecutionPanel />
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
import { ref, onMounted } from 'vue'
import WaypointItem from './AutonWaypointItem.vue'
import WaypointStore from './AutonWaypointStore.vue'
import AutonAddWaypointModal from './AutonAddWaypointModal.vue'
import AutonExecutionPanel from './AutonExecutionPanel.vue'
import ConfirmModal from './ConfirmModal.vue'
import { VueDraggable } from 'vue-draggable-plus'
import type { AutonWaypoint } from '@/types/waypoints'
import { useAutonomyStore } from '@/stores/autonomy'

const autonomyStore = useAutonomyStore()

const addModal = ref<InstanceType<typeof AutonAddWaypointModal> | null>(null)
const resetModal = ref<InstanceType<typeof ConfirmModal> | null>(null)

onMounted(() => {
  autonomyStore.fetchAll()
})

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
</script>

<style scoped>
.editor-column {
  display: flex;
  flex: 1;
  flex-direction: column;
  min-width: 0;
}

.waypoint-wrapper {
  scrollbar-gutter: stable;
  background-color: var(--view-bg);
}
</style>

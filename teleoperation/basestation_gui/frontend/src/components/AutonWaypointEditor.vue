<template>
  <div class="wrapper flex m-0 p-0 h-full w-full gap-2">
    <div class="editor-column">
      <div class="waypoint-header p-2 mb-2 flex justify-between items-center border-b">
        <h4 class="component-header">Waypoint Store</h4>
        <div class="flex gap-2 items-center">
          <button
            class="cmd-btn cmd-btn-danger cmd-btn-sm cmd-btn-icon-sm"
            data-testid="pw-reset-waypoints-btn"
            @click="resetModal?.open()"
            title="Reset waypoints"
          >
            <i class="bi bi-arrow-clockwise"></i>
          </button>
          <button class="cmd-btn cmd-btn-sm cmd-btn-success" data-testid="pw-add-from-map" @click="addModal?.open()">
            Add from Map
          </button>
        </div>
      </div>
      <div class="waypoint-wrapper p-2 rounded grow overflow-auto" data-testid="pw-waypoint-store-list">
        <WaypointStore
          v-for="(waypoint, index) in autonomyStore.waypoints"
          :key="waypoint.db_id || index"
          :waypoint="waypoint"
          :index="index"
          @add="autonomyStore.addToRoute"
          @delete="autonomyStore.deleteWaypoint"
          @update="handleStoreUpdate"
        />
      </div>
    </div>

    <div class="editor-column">
      <div class="waypoint-header p-2 mb-2 flex justify-between items-center border-b">
        <h4 class="component-header">Current Course</h4>
        <button
          class="cmd-btn cmd-btn-sm"
          :class="autonomyStore.allCostmapToggle ? 'cmd-btn-success' : 'cmd-btn-danger'"
          data-testid="pw-costmap-toggle-all"
          @click="autonomyStore.toggleAllCostmaps()"
        >
          All Costmaps
        </button>
      </div>
      <draggable
        v-model="autonomyStore.route"
        item-key="tag_id"
        handle=".drag-handle"
        ghost-class="drag-ghost"
        class="waypoint-wrapper p-2 rounded flex flex-col gap-1 grow overflow-auto"
        @end="autonomyStore.saveRoute()"
      >
        <template #item="{ element }">
          <WaypointItem
            :waypoint="element"
            @delete="autonomyStore.removeFromRoute(element)"
            @toggleCostmap="autonomyStore.toggleRouteCostmap"
          />
        </template>
      </draggable>
    </div>
  </div>

  <AutonAddWaypointModal ref="addModal" />

  <ConfirmModal
    ref="resetModal"
    modal-id="modalReset"
    title="Reset Waypoints"
    confirm-text="Reset"
    @confirm="autonomyStore.resetUserWaypoints()"
  >
    <p>This will clear all user-added waypoints and the current course.</p>
    <p class="text-muted mb-0">Default waypoints will be preserved.</p>
  </ConfirmModal>
</template>

<script lang="ts" setup>
import { ref, onMounted } from 'vue'
import WaypointItem from './AutonWaypointItem.vue'
import WaypointStore from './AutonWaypointStore.vue'
import AutonAddWaypointModal from './AutonAddWaypointModal.vue'
import ConfirmModal from './ConfirmModal.vue'
import draggable from 'vuedraggable'
import type { AutonWaypoint } from '@/types/waypoints'
import { useAutonomyStore } from '@/stores/autonomy'

const autonomyStore = useAutonomyStore()

const addModal = ref<InstanceType<typeof AutonAddWaypointModal> | null>(null)
const resetModal = ref<InstanceType<typeof ConfirmModal> | null>(null)

onMounted(() => {
  autonomyStore.fetchAll()
})

function handleStoreUpdate(waypoint: AutonWaypoint, index: number) {
  const existing = autonomyStore.waypoints[index]
  if (!existing || existing.db_id == null) return

  const fields: Partial<AutonWaypoint> = {}
  if (waypoint.name !== existing.name) fields.name = waypoint.name
  if (waypoint.lat !== existing.lat) fields.lat = waypoint.lat
  if (waypoint.lon !== existing.lon) fields.lon = waypoint.lon
  if (Object.keys(fields).length > 0) {
    autonomyStore.updateWaypoint(existing.db_id, fields)
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

.drag-ghost {
  background-color: var(--control-primary);
  border-style: dashed !important;
  opacity: 0.4;
}
</style>

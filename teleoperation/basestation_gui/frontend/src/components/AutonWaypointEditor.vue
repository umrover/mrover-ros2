<template>
  <div class="wrapper d-flex m-0 p-0 h-100 w-100 gap-2">
    <!-- Left Column: Waypoint Store (Inactive/All Waypoints) -->
    <div class="editor-column">
      <div class="waypoint-header p-2 mb-2 d-flex justify-content-between align-items-center border-bottom border-2">
        <h4 class="component-header m-0 p-0">Waypoint Store</h4>
        <div class="d-flex gap-2 align-items-center">
          <button
            class="btn btn-danger btn-sm border-2 cmd-btn-icon-sm"
            @click="openResetModal"
            title="Reset waypoints"
          >
            <i class="bi bi-arrow-clockwise"></i>
          </button>
          <button class="btn btn-sm btn-success border-2" data-testid="pw-add-from-map" @click="openModal()">
            Add from Map
          </button>
        </div>
      </div>
      <div class="waypoint-wrapper p-2 rounded flex-grow-1 overflow-auto" data-testid="pw-waypoint-store-list">
        <WaypointStore
          v-for="(waypoint, index) in waypoints"
          :key="waypoint.id || index"
          :waypoint="waypoint"
          :index="index"
          @add="addToRoute"
          @delete="deleteFromStore"
          @update="updateStoreWaypoint"
        />
      </div>
    </div>

    <!-- Right Column: Active Route -->
    <div class="editor-column">
      <div class="waypoint-header p-2 mb-2 d-flex justify-content-between align-items-center border-bottom border-2">
        <h4 class="component-header m-0 p-0">Current Course</h4>
        <button
          class="btn btn-sm border-2"
          :class="allCostmapToggle ? 'btn-success' : 'btn-danger'"
          @click="toggleAllCostmaps"
        >
          All Costmaps
        </button>
      </div>
      <draggable
        v-model="currentRoute"
        item-key="id"
        handle=".drag-handle"
        ghost-class="drag-ghost"
        class="waypoint-wrapper p-2 rounded d-flex flex-column gap-1 flex-grow-1 overflow-auto"
      >
        <template #item="{ element }">
          <WaypointItem
            :waypoint="element"
            @delete="deleteFromRoute(element)"
            @toggleCostmap="toggleRouteCostmap"
          />
        </template>
      </draggable>
    </div>
  </div>

  <!-- Add Waypoint Modal -->
  <Teleport to="body">
    <div class="modal fade" id="modalWypt" tabindex="-1" role="dialog" data-testid="pw-waypoint-modal">
      <div class="modal-dialog modal-dialog-centered" role="document">
        <div class="modal-content">
          <div class="modal-header">
            <h5 class="modal-title">Add Waypoint</h5>
            <button type="button" class="btn-close" @click="closeModal"></button>
          </div>
          <div class="modal-body">
            <div class="row g-3">
              <div class="col-md-6">
                <label for="waypointname" class="form-label">Name:</label>
                <input
                  class="form-control"
                  id="waypointname"
                  data-testid="pw-waypoint-name-input"
                  v-model="modalWypt.name"
                />
              </div>
              <div class="col-md-6">
                <label for="waypointid" class="form-label">Tag ID:</label>
                <input
                  v-if="modalWypt.type == 1"
                  class="form-control"
                  id="waypointid"
                  v-model="modalWypt.id"
                  type="number"
                  max="249"
                  min="0"
                  step="1"
                />
                <input
                  v-else
                  class="form-control"
                  id="waypointid"
                  type="number"
                  placeholder="-1"
                  disabled
                />
              </div>
              <div class="col-12">
                <label class="form-label">Type:</label>
                <select class="form-select" v-model="modalWypt.type">
                  <option value="0">No Search</option>
                  <option value="1">Post</option>
                  <option value="2">Mallet</option>
                  <option value="3">Water Bottle</option>
                  <option value="4">Rock Pick</option>
                </select>
              </div>
            </div>
          </div>
          <div class="modal-footer">
            <button
              type="button"
              class="btn btn-secondary border-2"
              data-testid="pw-add-waypoint-submit"
              @click="saveNewWaypoint"
            >
              Add Waypoint
            </button>
          </div>
        </div>
      </div>
    </div>

    <!-- Reset Confirmation Modal -->
    <div class="modal fade" id="modalReset" tabindex="-1" role="dialog">
      <div class="modal-dialog modal-dialog-centered" role="document">
        <div class="modal-content">
          <div class="modal-header">
            <h5 class="modal-title">Reset Waypoints</h5>
            <button type="button" class="btn-close" @click="closeResetModal"></button>
          </div>
          <div class="modal-body">
            <p>This will clear all user-added waypoints and the current course.</p>
            <p class="text-muted mb-0">Default waypoints will be preserved.</p>
          </div>
          <div class="modal-footer">
            <button type="button" class="btn btn-secondary border-2" @click="closeResetModal">
              Cancel
            </button>
            <button type="button" class="btn btn-danger border-2" @click="confirmReset">
              Reset
            </button>
          </div>
        </div>
      </div>
    </div>
  </Teleport>
</template>

<script lang="ts">
import WaypointItem from './AutonWaypointItem.vue'
import WaypointStore from './AutonWaypointStore.vue'
import draggable from 'vuedraggable'

import L from 'leaflet'
import { defineComponent } from 'vue'
import { Modal } from 'bootstrap'
import type { AutonWaypoint } from '@/types/waypoints'
import { waypointsAPI } from '@/utils/api'
import { useAutonomyStore } from '@/stores/autonomy'

export default defineComponent({
  components: {
    WaypointItem,
    WaypointStore,
    draggable,
  },

  setup() {
    const autonomyStore = useAutonomyStore()
    return { autonomyStore }
  },

  data() {
    return {
      waypoints: [] as AutonWaypoint[],
      currentRoute: [] as AutonWaypoint[],
      allCostmapToggle: true,

      modal: null as Modal | null,
      resetModal: null as Modal | null,
      modalWypt: {
        name: '',
        id: -1,
        type: 0,
        lat: 0,
        lon: 0,
        enable_costmap: true,
      },

      nextAvailableTagId: 8,
    }
  },

  computed: {
    clickPoint() {
      return this.autonomyStore.clickPoint
    },
  },

  watch: {
    // Sync Waypoint Store with Backend
    waypoints: {
      async handler(newList: AutonWaypoint[]) {
        // Update map visualization
        const mapPoints = newList.map(waypoint => ({
          latLng: L.latLng(waypoint.lat, waypoint.lon),
          name: waypoint.name
        }))
        this.autonomyStore.setWaypointList(mapPoints)

        // Save to backend
        try {
          await waypointsAPI.saveAuton(newList)
        } catch (error) {
          console.error('Failed to save auton waypoints:', error)
        }
      },
      deep: true,
    },

    // Sync Active Route with Backend
    currentRoute: {
      async handler(newRoute: AutonWaypoint[]) {
        // Update map visualization
        const mapPoints = newRoute.map(waypoint => ({
          latLng: L.latLng(waypoint.lat, waypoint.lon),
          name: waypoint.name,
          id: waypoint.id,
          type: waypoint.type,
          enable_costmap: waypoint.enable_costmap
        }))
        this.autonomyStore.setRoute(mapPoints)

        // Save to backend
        try {
          await waypointsAPI.saveCurrentAutonCourse(newRoute)
        } catch (error) {
          console.error('Failed to save current auton course:', error)
        }
      },
      deep: true,
    },

    // Apply costmap toggle to all route waypoints
    allCostmapToggle(newState: boolean) {
      this.currentRoute.forEach((wp: AutonWaypoint) => {
        wp.enable_costmap = newState
      })
    },
  },

  async mounted() {
    this.modal = new Modal('#modalWypt', {})
    this.resetModal = new Modal('#modalReset', {})
    await this.fetchData()
  },

  methods: {
    async fetchData() {
      try {
        // Fetch Store Waypoints
        const autonData = await waypointsAPI.getAuton()
        if (autonData.status === 'success') {
          this.waypoints = autonData.waypoints || []

          // Calculate next available tag ID from existing waypoints
          const maxTagId = this.waypoints.reduce((max, wp) => {
            return wp.id > max ? wp.id : max
          }, 7)
          this.nextAvailableTagId = maxTagId + 1
        }

        // Fetch Active Route
        const courseData = await waypointsAPI.getCurrentAutonCourse()
        if (courseData.status === 'success') {
          this.currentRoute = courseData.course || []

          // Mark items in route as "in_route" in the store list for visual feedback
          this.waypoints.forEach(wp => {
             wp.in_route = this.currentRoute.some(
               r => r.name === wp.name && r.id === wp.id && r.type === wp.type
             )
          })
        }
      } catch (error) {
        console.error('Failed to load waypoints:', error)
      }
    },

    // --- Route Management ---

    addToRoute(waypoint: AutonWaypoint) {
      // Deep copy to allow independent modification (e.g., enable_costmap) in route vs store
      const newPoint = { ...waypoint, enable_costmap: this.allCostmapToggle }

      // Visual feedback in store list
      const storeIndex = this.waypoints.findIndex(w => w === waypoint)
      if (storeIndex !== -1) {
        const storeWaypoint = this.waypoints[storeIndex]
        if (storeWaypoint) {
          storeWaypoint.in_route = true
        }
      }

      this.currentRoute.push(newPoint)
    },

    deleteFromRoute(waypoint: AutonWaypoint) {
      const index = this.currentRoute.indexOf(waypoint)
      if (index > -1) {
        this.currentRoute.splice(index, 1)
      }

      // Update visual feedback in store
      // Check if this type of waypoint still exists in the route elsewhere
      const stillInRoute = this.currentRoute.some(
         r => r.name === waypoint.name && r.id === waypoint.id && r.type === waypoint.type
      )

      if (!stillInRoute) {
        const storeIndex = this.waypoints.findIndex(
           w => w.name === waypoint.name && w.id === waypoint.id && w.type === waypoint.type
        )
        if (storeIndex !== -1) {
          const storeWaypoint = this.waypoints[storeIndex]
          if (storeWaypoint) {
            storeWaypoint.in_route = false
          }
        }
      }
    },

    toggleRouteCostmap({ waypoint, enable_costmap }: { waypoint: AutonWaypoint, enable_costmap: boolean }) {
      waypoint.enable_costmap = enable_costmap
    },

    toggleAllCostmaps() {
      this.allCostmapToggle = !this.allCostmapToggle
    },

    // --- Store Management ---

    saveNewWaypoint() {
      this.modalWypt.lat = this.clickPoint.lat
      this.modalWypt.lon = this.clickPoint.lon

      // Assign next available tag ID if this is not a Post type (type 1 allows user to set tag_id)
      if (this.modalWypt.type !== 1) {
        this.modalWypt.id = this.nextAvailableTagId
        this.nextAvailableTagId++
      }

      // Add to store (default deletable=true)
      this.waypoints.push({ ...this.modalWypt, enable_costmap: true })

      // Reset modal
      this.modalWypt = {
        name: '',
        id: -1,
        type: 0,
        lat: 0,
        lon: 0,
        enable_costmap: true,
      }
      this.closeModal()
    },

    async deleteFromStore(index: number) {
      const wp = this.waypoints[index]
      if (!wp) return

      // Optimistic UI update
      this.waypoints.splice(index, 1)

      if (wp.db_id && wp.deletable) {
        try {
          await waypointsAPI.deleteAutonWaypoint(wp)
        } catch (error) {
          console.error('Failed to delete waypoint:', error)
          // Revert on failure (optional, but good practice)
          this.waypoints.splice(index, 0, wp)
        }
      }
    },

    updateStoreWaypoint(waypoint: AutonWaypoint, index: number) {
      const storeWaypoint = this.waypoints[index]
      if (storeWaypoint) {
        storeWaypoint.lat = waypoint.lat
        storeWaypoint.lon = waypoint.lon
      }
    },

    // --- Modal ---
    openModal() {
      this.modal?.show()
    },
    closeModal() {
      if (document.activeElement instanceof HTMLElement) {
        document.activeElement.blur()
      }
      this.modal?.hide()
    },

    // --- Reset ---
    openResetModal() {
      this.resetModal?.show()
    },
    closeResetModal() {
      this.resetModal?.hide()
    },
    async confirmReset() {
      try {
        await waypointsAPI.clearAuton()
        await this.fetchData()
      } catch (error) {
        console.error('Failed to reset waypoints:', error)
      } finally {
        this.closeResetModal()
      }
    }
  },
})
</script>

<style scoped>
.editor-column {
  display: flex;
  flex-direction: column;
  flex: 1;
  min-width: 0;
}

.waypoint-wrapper {
  background-color: var(--view-bg);
  scrollbar-gutter: stable;
}

.drag-ghost {
  opacity: 0.4;
  background-color: var(--bs-secondary-bg);
  border-style: dashed !important;
}
</style>

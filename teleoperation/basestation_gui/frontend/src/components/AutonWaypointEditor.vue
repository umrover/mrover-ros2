<template>
  <div class="wrapper d-flex m-0 p-0 h-100 w-100 gap-2">
    <!-- Left Column: Waypoint Store (Inactive/All Waypoints) -->
    <div class="d-flex flex-column w-100">
      <div class="waypoint-header p-1 d-flex justify-content-between align-items-center">
        <h3 class="m-0 p-0">Waypoints</h3>
        <button class="btn btn-success" @click="openModal()">
          Add from Map
        </button>
      </div>
      <div class="waypoint-wrapper overflow-y-scroll flex-grow-1">
        <WaypointStore
          v-for="(waypoint, index) in waypoints"
          :key="waypoint.tag_id || index"
          :waypoint="waypoint"
          :index="index"
          @add="addToRoute"
          @delete="deleteFromStore"
          @update="updateStoreWaypoint"
        />
      </div>
    </div>

    <!-- Right Column: Active Route & Controls -->
    <div class="d-flex flex-column w-100">
      <!-- Controls Grid -->
      <div class="datagrid m-0 p-0">
        <FeedbackButton
          ref="autonCheckbox"
          class="auton-checkbox"
          :name="'Autonomy Mode'"
          :checked="autonEnabled"
          :disabled="teleopEnabled"
          :action="autonAction"
          @toggle="handleAutonToggle"
        />
        <div class="stats">
          <VelocityReading />
        </div>
        <FeedbackButton
          ref="teleopCheckbox"
          class="teleop-checkbox"
          :name="'Teleop Controls'"
          :checked="teleopEnabled"
          :action="teleopAction"
          @toggle="handleTeleopToggle"
        />
        <FeedbackButton
          ref="costmapCheckbox"
          class="costmap-checkbox"
          :name="'All Costmaps'"
          :checked="allCostmapToggle"
          @toggle="handleCostmapToggle"
        />
      </div>

      <!-- Active Route List -->
      <h3 class="m-0 p-0">Current Course</h3>
      <div class="waypoint-wrapper overflow-y-scroll d-flex flex-column gap-2 flex-grow-1">
        <WaypointItem
          v-for="(waypoint, index) in currentRoute"
          :key="index"
          :waypoint="waypoint"
          @delete="deleteFromRoute(waypoint)"
          @toggleCostmap="toggleRouteCostmap"
        />
      </div>
    </div>
  </div>

  <!-- Add Waypoint Modal -->
  <Teleport to="body">
    <div class="modal fade" id="modalWypt" tabindex="-1" role="dialog">
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
                  v-model="modalWypt.name"
                />
              </div>
              <div class="col-md-6">
                <label for="waypointid" class="form-label">Tag ID:</label>
                <input
                  v-if="modalWypt.type == 1"
                  class="form-control"
                  id="waypointid"
                  v-model="modalWypt.tag_id"
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
              class="btn btn-secondary"
              @click="saveNewWaypoint"
            >
              Add Waypoint
            </button>
          </div>
        </div>
      </div>
    </div>
  </Teleport>
</template>

<script lang="ts">
import FeedbackButton from './FeedbackButton.vue'
import VelocityReading from './VelocityReading.vue'
import WaypointItem from './AutonWaypointItem.vue'
import WaypointStore from './AutonWaypointStore.vue'

import L from 'leaflet'
import { defineComponent } from 'vue'
import { Modal } from 'bootstrap'
import type { AutonWaypoint } from '@/types/waypoints'
import { waypointsAPI, autonAPI } from '@/utils/api'
import { useWebsocketStore } from '@/stores/websocket'
import { useAutonomyStore } from '@/stores/autonomy'

export default defineComponent({
  components: {
    WaypointItem,
    FeedbackButton,
    VelocityReading,
    WaypointStore,
  },

  setup() {
    const websocketStore = useWebsocketStore()
    const autonomyStore = useAutonomyStore()
    return { websocketStore, autonomyStore }
  },

  emits: ['toggleTeleop'],

  data() {
    return {
      // The "Store": list of all available waypoints
      waypoints: [] as AutonWaypoint[],

      // The "Active Route": subset of waypoints currently being navigated
      currentRoute: [] as AutonWaypoint[],

      modal: null as Modal | null,
      modalWypt: {
        name: '',
        tag_id: -1,
        type: 0,
        lat: 0,
        lon: 0,
        enable_costmap: true,
      },

      allCostmapToggle: true,
      saveWaypointsTimer: null as ReturnType<typeof setTimeout> | null,
      saveRouteTimer: null as ReturnType<typeof setTimeout> | null,
    }
  },

  computed: {
    navMessage() {
      return this.websocketStore.messages['nav']
    },
    autonEnabled() {
      return this.autonomyStore.autonEnabled
    },
    teleopEnabled() {
      return this.autonomyStore.teleopEnabled
    },
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
          tag_id: waypoint.tag_id,
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
  },

  async mounted() {
    this.modal = new Modal('#modalWypt', {})
    await this.fetchData()
  },

  methods: {
    async fetchData() {
      try {
        // Fetch Store Waypoints
        const autonData = await waypointsAPI.getAuton()
        if (autonData.status === 'success') {
          this.waypoints = autonData.waypoints || []
        }

        // Fetch Active Route
        const courseData = await waypointsAPI.getCurrentAutonCourse()
        if (courseData.status === 'success') {
          this.currentRoute = courseData.course || []

          // Mark items in route as "in_route" in the store list for visual feedback
          this.waypoints.forEach(wp => {
             wp.in_route = this.currentRoute.some(
               r => r.name === wp.name && r.tag_id === wp.tag_id && r.type === wp.type
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
         r => r.name === waypoint.name && r.tag_id === waypoint.tag_id && r.type === waypoint.type
      )

      if (!stillInRoute) {
        const storeIndex = this.waypoints.findIndex(
           w => w.name === waypoint.name && w.tag_id === waypoint.tag_id && w.type === waypoint.type
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

    handleCostmapToggle(newState: boolean) {
      this.allCostmapToggle = newState
      this.currentRoute.forEach((wp: AutonWaypoint) => {
        wp.enable_costmap = newState
      })
    },

    // --- Store Management ---

    saveNewWaypoint() {
      this.modalWypt.lat = this.clickPoint.lat
      this.modalWypt.lon = this.clickPoint.lon

      this.waypoints.push({ ...this.modalWypt, enable_costmap: true })

      this.modalWypt = {
        name: '',
        tag_id: -1,
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

    // --- Auton Control ---

    autonAction(newState: boolean) {
      const waypoints = newState
        ? this.currentRoute.map((waypoint: AutonWaypoint) => ({
            latitude_degrees: waypoint.lat,
            longitude_degrees: waypoint.lon,
            tag_id: waypoint.tag_id,
            type: waypoint.type,
            enable_costmap: waypoint.enable_costmap,
          }))
        : []

      return autonAPI.enable(newState, waypoints)
    },

    handleAutonToggle(newState: boolean) {
      this.autonomyStore.setAutonMode(newState)
    },

    teleopAction(newState: boolean) {
      return autonAPI.enableTeleop(newState)
    },

    async handleTeleopToggle(newState: boolean) {
      this.autonomyStore.setTeleopMode(newState)
      this.$emit('toggleTeleop', newState)

      if (newState && this.autonEnabled) {
        await this.autonAction(false)
        this.autonomyStore.setAutonMode(false)
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
    }
  },
})
</script>

<style scoped>
.datagrid {
  display: grid;
  grid-gap: 6px;
  grid-template-columns: 65% auto;
  grid-template-rows: auto auto;
  grid-template-areas:
    'auton-check stats'
    'teleop-check stats'
    'costmap-check stats';
  font-family: sans-serif;
  padding-bottom: 10px;
}

.waypoint-wrapper {
  background-color: var(--view-bg);
  padding: 8px;
  border-radius: 8px;
}

.teleop-checkbox { grid-area: teleop-check; width: 100%; }
.costmap-checkbox { grid-area: costmap-check; width: 100%; }
.stats { grid-area: stats; }
.auton-checkbox { grid-area: auton-check; }
</style>
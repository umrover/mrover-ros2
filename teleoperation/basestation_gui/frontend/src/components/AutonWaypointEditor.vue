<template>
  <div class="wrapper d-flex m-0 p-0 h-100 w-100 gap-2">
    <div class="d-flex flex-column w-100">
      <div class="waypoint-header p-1">
        <h3 class="m-0 p-0">Waypoints</h3>
        <button class="btn btn-success" @click="openModal()">
          Add from Map
        </button>
      </div>
      <div class="waypoint-wrapper overflow-y-scroll">
        <WaypointStore
          v-for="(waypoint, index) in waypoints"
          :key="index"
          :waypoint="waypoint"
          :index="index"
          @add="addItem"
          @delete="deleteMapWaypoint"
        />
      </div>
    </div>
    <div class="d-flex flex-column w-100">
      <div class="datagrid m-0 p-0">
        <FeedbackButton
          ref="autonCheckbox"
          class="auton-checkbox"
          :name="'Autonomy Mode'"
          :checked="autonEnabled"
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
      <h3 class="m-0 p-0">Current Course</h3>
      <div class="waypoint-wrapper overflow-y-scroll d-flex flex-column gap-2">
        <WaypointItem
          v-for="(waypoint, index) in currentRoute"
          :key="index"
          :waypoint="waypoint"
          @delete="deleteItem(waypoint)"
          @toggleCostmap="toggleCostmap"
        />
      </div>
    </div>
  </div>

  <div class="modal fade" id="modalWypt" tabindex="-1" role="dialog">
    <div class="modal-dialog modal-dialog-centered" role="document">
      <div class="modal-content">
        <div class="modal-body">
          <div class="row">
            <div class="form-group col-md-6">
              <label for="waypointname">Name:</label>
              <input
                class="form-control"
                id="waypointname"
                v-model="modalWypt.name"
              />
            </div>
            <div class="form-group col-md-6">
              <label for="waypointid">Tag ID:</label>
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
                step="1"
                disabled
              />
            </div>
            <select class="form-select my-3" v-model="modalWypt.type">
              <option value="0" selected>No Search</option>
              <option value="1">Post</option>
              <option value="2">Mallet</option>
              <option value="3">Water Bottle</option>
              <option value="4">Rock Pick</option>
            </select>
          </div>
        </div>
        <div class="modal-footer">
          <button
            type="button"
            class="btn btn-secondary"
            @click="addMapWaypoint()"
          >
            Add Waypoint
          </button>
        </div>
      </div>
    </div>
  </div>
</template>

<script lang="ts">
import FeedbackButton from './FeedbackButton.vue'
import VelocityReading from './VelocityReading.vue'
import WaypointItem from './AutonWaypointItem.vue'
import WaypointStore from './AutonWaypointStore.vue'

import L from 'leaflet'
import { reactive, defineComponent } from 'vue'
import { Modal } from 'bootstrap'
import type { AutonWaypoint, StoreWaypoint } from '@/types/waypoints'
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
      waypoints: [
        {
          name: 'No Search 1',
          id: -1,
          type: 0,
          lat: 0,
          lon: 0,
          enable_costmap: true,
        },
        {
          name: 'No Search 2',
          id: -1,
          type: 0,
          lat: 0,
          lon: 0,
          enable_costmap: true,
        },
        {
          name: 'Post 1',
          id: 1,
          type: 1,
          lat: 0,
          lon: 0,
          enable_costmap: true,
        },
        {
          name: 'Post 2',
          id: 2,
          type: 1,
          lat: 0,
          lon: 0,
          enable_costmap: true,
        },
        {
          name: 'Post 3',
          id: 3,
          type: 1,
          lat: 0,
          lon: 0,
          enable_costmap: true,
        },
        {
          name: 'Mallet',
          id: -1,
          type: 2,
          lat: 0,
          lon: 0,
          enable_costmap: true,
        },
        {
          name: 'Water Bottle',
          id: -1,
          type: 3,
          lat: 0,
          lon: 0,
          enable_costmap: true,
        },
        {
          name: 'Rock Pick',
          id: -1,
          type: 4,
          lat: 0,
          lon: 0,
          enable_costmap: true,
        },
      ] as AutonWaypoint[],

      modal: null as Modal | null,
      modalWypt: {
        name: '',
        id: -1,
        type: 0,
        lat: 0,
        lon: 0,
        enable_costmap: true,
      },

      allCostmapToggle: true,

      route: reactive([] as AutonWaypoint[]),

      currentRoute: [] as AutonWaypoint[],
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
    }
  },

  watch: {
    waypoints: {
      async handler(newList: AutonWaypoint[]) {
        const waypoints = newList.map(waypoint => {
          const lat = waypoint.lat
          const lon = waypoint.lon
          return { latLng: L.latLng(lat, lon), name: waypoint.name }
        })
        this.setWaypointList(waypoints)

        try {
          await waypointsAPI.saveAuton(newList)
        } catch (error) {
          console.error('Failed to save auton waypoints:', error)
        }
      },
      deep: true,
    },

    currentRoute: {
      async handler(newRoute: AutonWaypoint[]) {
        const waypoints = newRoute.map(waypoint => {
          const lat = waypoint.lat
          const lon = waypoint.lon
          return { latLng: L.latLng(lat, lon), name: waypoint.name }
        })
        this.setRoute(waypoints)

        try {
          await waypointsAPI.saveCurrentAutonCourse(this.currentRoute)
        } catch (error) {
          console.error('Failed to save current auton course:', error)
        }
      },
      deep: true,
    },

  },

  mounted() {
    this.modal = new Modal('#modalWypt', {})
  },

  async created() {
    setTimeout(async () => {
      try {
        const autonData = await waypointsAPI.getAuton()
        if (autonData.status === 'success' && autonData.waypoints && autonData.waypoints.length > 0) {
          this.waypoints = autonData.waypoints
          const waypoints = autonData.waypoints.map(
            (waypoint: { lat: number; lon: number; name: string }) => ({
              latLng: L.latLng(waypoint.lat, waypoint.lon),
              name: waypoint.name
            })
          )
          this.setWaypointList(waypoints)
        }

        const courseData = await waypointsAPI.getCurrentAutonCourse()
        if (courseData.status === 'success' && courseData.course) {
          this.currentRoute = courseData.course
        }
      } catch (error) {
        console.error('Failed to load auton waypoints:', error)
      }
    }, 250)
  },

  methods: {
    setRoute(route: StoreWaypoint[]) {
      this.autonomyStore.setRoute(route)
    },
    setWaypointList(list: StoreWaypoint[]) {
      this.autonomyStore.setWaypointList(list)
    },
    setAutonMode(mode: boolean) {
      this.autonomyStore.setAutonMode(mode)
    },
    setTeleopMode(mode: boolean) {
      this.autonomyStore.setTeleopMode(mode)
    },

    autonAction(newState: boolean) {
      const waypoints = newState
        ? this.currentRoute.map((waypoint: AutonWaypoint) => ({
            latitude_degrees: waypoint.lat,
            longitude_degrees: waypoint.lon,
            tag_id: waypoint.id,
            type: waypoint.type,
            enable_costmap: waypoint.enable_costmap,
          }))
        : []

      return autonAPI.enable(newState, waypoints)
    },

    handleAutonToggle(newState: boolean) {
      this.setAutonMode(newState)
    },

    async deleteItem(waypoint: AutonWaypoint) {
      waypoint.in_route = false
      const index = this.route.indexOf(waypoint)
      this.route.splice(index, 1)
      this.currentRoute.splice(this.currentRoute.indexOf(waypoint), 1)

      try {
        await waypointsAPI.deleteAutonWaypoint(waypoint)
      } catch (error) {
        console.error('Failed to delete auton waypoint:', error)
      }
    },

    toggleCostmap({
      waypoint,
      enable_costmap,
    }: {
      waypoint: AutonWaypoint
      enable_costmap: boolean
    }) {
      waypoint.enable_costmap = enable_costmap
    },

    handleCostmapToggle(newState: boolean) {
      this.allCostmapToggle = newState
      this.currentRoute.forEach((wp: AutonWaypoint) => {
        wp.enable_costmap = newState
      })
    },

    addItem: function (waypoint: AutonWaypoint) {
      if (!waypoint.in_route) {
        waypoint['enable_costmap'] = this.allCostmapToggle
        this.route.push(waypoint)
        this.currentRoute.push(waypoint)
        waypoint.in_route = true
      }
    },

    openModal: function () {
      if (this.modal) {
        this.modal.show()
      }
    },

    addMapWaypoint: function () {
      this.modalWypt.lat = this.clickPoint.lat
      this.modalWypt.lon = this.clickPoint.lon
      this.waypoints.push(this.modalWypt)
      this.modalWypt = {
        name: '',
        id: -1,
        type: 0,
        lat: 0,
        lon: 0,
        enable_costmap: true,
      }
      if (this.modal) {
        this.modal.hide()
      }
    },

    deleteMapWaypoint: function (index: number) {
      this.waypoints.splice(index, 1)
    },

    teleopAction(newState: boolean) {
      return autonAPI.enableTeleop(newState)
    },

    handleTeleopToggle(newState: boolean) {
      this.setTeleopMode(newState)
      this.$emit('toggleTeleop', newState)
    },
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
  flex: 1;
  overflow-y: auto;
  background-color: #dddddd;
  padding: 8px;
  border-radius: 8px;
}

.waypoint-header {
  display: inline-flex;
  width: 100%;
  justify-content: space-between;
  align-items: center;
}

.teleop-checkbox {
  grid-area: teleop-check;
  width: 100%;
}

.costmap-checkbox {
  grid-area: costmap-check;
  width: 100%;
}

.stats {
  grid-area: stats;
}

.auton-checkbox {
  grid-area: auton-check;
}

.odom {
  grid-area: odom;
}
</style>

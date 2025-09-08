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
          :key="waypoint"
          :waypoint="waypoint"
          :index="index"
          @add="addItem"
          @delete="deleteMapWaypoint"
        />
      </div>
    </div>
    <div class="d-flex flex-column w-100">
      <div class="datagrid m-0 p-0">
        <AutonModeCheckbox
          ref="autonCheckbox"
          class="auton-checkbox"
          :name="autonButtonText"
          :color="autonButtonColor"
          @toggle="toggleAutonMode($event)"
        />
        <div class="stats">
          <VelocityReading />
        </div>
        <Checkbox
          ref="teleopCheckbox"
          class="teleop-checkbox"
          :name="'Teleop Controls'"
          :width="220"
          @toggle="toggleTeleopMode($event)"
        />
        <Checkbox
          ref="costmapCheckbox"
          class="costmap-checkbox"
          :name="'Kill All Costmaps'"
          :width="220"
          @toggle="toggleAllCostmaps"
        />
      </div>
      <h3 class="m-0 p-0">Current Course</h3>
      <div class="waypoint-wrapper overflow-y-scroll d-flex flex-column gap-2">
        <WaypointItem
          v-for="waypoint in currentRoute"
          :key="waypoint"
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
import AutonModeCheckbox from './AutonModeCheckbox.vue'
import Checkbox from './BasicCheckbox.vue'
import VelocityReading from './VelocityReading.vue'
import WaypointItem from './AutonWaypointItem.vue'
import WaypointStore from './AutonWaypointStore.vue'
import Vuex from 'vuex'
const { mapState, mapActions, mapMutations, mapGetters } = Vuex
import L from 'leaflet'
import { reactive, defineComponent } from 'vue'
import { Modal } from 'bootstrap'
import type { Waypoint } from '../types/waypoint'
import type { WebSocketState } from '@/types/websocket'

let auton_publish_interval: number

export default defineComponent({
  components: {
    WaypointItem,
    AutonModeCheckbox,
    Checkbox,
    VelocityReading,
    WaypointStore,
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
      ] as Waypoint[],

      modal: null as Modal | null,
      modalWypt: {
        name: '',
        id: -1,
        type: 0,
        lat: 0,
        lon: 0,
        enable_costmap: true,
      },

      teleopEnabledCheck: false,
      allCostmapToggle: true,

      route: reactive([]),

      currentRoute: [],

      autonButtonColor: 'btn-danger',

      roverStuck: false,
      waitingForNavResponse: false,
    }
  },
  computed: {
    ...mapState('websocket', {
      navMessage: (state: WebSocketState) => state.messages['nav'],
    }),
    ...mapGetters('autonomy', {
      autonEnabled: 'autonEnabled',
      teleopEnabled: 'teleopEnabled',
      clickPoint: 'clickPoint',
    }),

    autonButtonText: function () {
      return this.autonButtonColor == 'btn-warning'
        ? 'Setting to ' + this.autonEnabled
        : 'Autonomy Mode'
    },
  },

  watch: {
    waypoints: {
      handler(newList: Waypoint[]) {
        const waypoints = newList.map(waypoint => {
          const lat = waypoint.lat
          const lon = waypoint.lon
          return { latLng: L.latLng(lat, lon), name: waypoint.name }
        })
        this.setWaypointList(waypoints)
        this.$store.dispatch('websocket/sendMessage', {
          id: 'waypoints',
          message: {
            type: 'save_auton_waypoint_list',
            data: newList,
          },
        })
      },
      deep: true,
    },

    currentRoute: {
      handler(newRoute: Waypoint[]) {
        const waypoints = newRoute.map(waypoint => {
          const lat = waypoint.lat
          const lon = waypoint.lon
          return { latLng: L.latLng(lat, lon), name: waypoint.name }
        })
        this.setRoute(waypoints)

        this.$store.dispatch('websocket/sendMessage', {
          id: 'waypoints',
          message: {
            type: 'save_current_auton_course',
            data: this.currentRoute,
          },
        })
      },
      deep: true,
    },

    navMessage(msg) {
      if (msg.type == 'nav_state') {
        // If still waiting for nav...
        if (
          (msg.state == 'OffState' && this.autonEnabled) ||
          (msg.state !== 'OffState' && !this.autonEnabled) ||
          (msg.state == 'DoneState' && !this.autonEnabled)
        ) {
          return
        }
        this.waitingForNavResponse = false
        this.autonButtonColor = this.autonEnabled ? 'btn-success' : 'btn-danger'
      } else if (msg.type == 'get_auton_waypoint_list') {
        // Get waypoints from server on page load
        console.log(msg)
        if (msg.data.length > 0) this.waypoints = msg.data
        const waypoints = msg.data.map(
          (waypoint: { lat: number; lon: number; name: string }) => {
            const lat = waypoint.lat
            const lon = waypoint.lon
            return { latLng: L.latLng(lat, lon), name: waypoint.name }
          },
        )
        this.setWaypointList(waypoints)
      }
      if (msg.type == 'get_current_auton_course') {
        // console.log("here2 before", this.currentRoute)
        this.currentRoute = msg.data
        // console.log("here2", this.currentRoute)
      }
    },
  },

  mounted() {
    this.modal = new Modal('#modalWypt', {})
  },

  beforeUnmount: function () {
    window.clearInterval(auton_publish_interval)
    this.autonEnabled = false
    this.sendAutonCommand()
  },

  created: function () {
    auton_publish_interval = window.setInterval(() => {
      if (this.waitingForNavResponse) {
        this.sendAutonCommand()
      }
    }, 1000)
    window.setTimeout(() => {
      // Timeout so websocket will be initialized
      this.$store.dispatch('websocket/sendMessage', {
        id: 'waypoints',
        message: {
          type: 'get_auton_waypoint_list',
        },
      })
      this.$store.dispatch('websocket/sendMessage', {
        id: 'waypoints',
        message: {
          type: 'get_current_auton_course',
        },
      })
    }, 1000)
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    ...mapMutations('autonomy', {
      setRoute: 'setRoute',
      setWaypointList: 'setWaypointList',
      setAutonMode: 'setAutonMode',
      setTeleopMode: 'setTeleopMode',
    }),

    sendAutonCommand() {
      if (this.autonEnabled) {
        this.$store.dispatch('websocket/sendMessage', {
          id: 'auton',
          message: {
            type: 'auton_enable',
            enabled: true,
            waypoints: this.currentRoute.map((waypoint: Waypoint) => {
              const lat = waypoint.lat
              const lon = waypoint.lon
              // Return a GPSWaypoint.msg formatted object for each
              return {
                latitude_degrees: lat,
                longitude_degrees: lon,
                tag_id: waypoint.id,
                type: waypoint.type,
                enable_costmap: waypoint.enable_costmap,
              }
            }),
          },
        })
      } else {
        //if auton's not enabled, send an empty message
        this.$store.dispatch('websocket/sendMessage', {
          id: 'auton',
          message: {
            type: 'auton_enable',
            enabled: false,
            waypoints: [],
          },
        })
      }
    },

    deleteItem: function (waypoint: Waypoint) {
      waypoint.in_route = false
      const index = this.route.indexOf(waypoint)
      this.route.splice(index, 1)
      this.currentRoute.splice(this.currentRoute.indexOf(waypoint), 1)
      this.$store.dispatch('websocket/sendMessage', {
        id: 'waypoints',
        message: {
          type: 'delete_auton_waypoint_from_course',
          data: waypoint,
        },
      })
    },

    toggleCostmap({ waypoint, enable_costmap }: { waypoint: Waypoint; enable_costmap: boolean }) {
      waypoint.enable_costmap = enable_costmap
    },

    toggleAllCostmaps() {
      this.allCostmapToggle = !this.allCostmapToggle
      this.waypoints.forEach((wp: Waypoint) => {
        wp.enable_costmap = this.allCostmapToggle
      })
    },

    addItem: function (waypoint: Waypoint) {
      if (!waypoint.in_route) {
        waypoint['enable_costmap'] = waypoint.enable_costmap ?? false
        this.route.push(waypoint)
        this.currentRoute.push(waypoint)
        waypoint.in_route = true
      }
    },

    openModal: function () {
      this.modal.show()
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
      this.modal.hide()
    },

    deleteMapWaypoint: function (index: number) {
      this.waypoints.splice(index, 1)
    },

    toggleAutonMode: function (val: boolean) {
      this.setAutonMode(val)
      // This will trigger the yellow "waiting for nav" state of the checkbox
      this.autonButtonColor = 'btn-warning'
      this.waitingForNavResponse = true
    },

    toggleTeleopMode: function () {
      this.teleopEnabledCheck = !this.teleopEnabledCheck
      this.$store.dispatch('websocket/sendMessage', {
        id: 'auton',
        message: {
          type: 'teleop_enable',
          enabled: this.teleopEnabledCheck,
        },
      })
      this.$emit('toggleTeleop', this.teleopEnabledCheck)
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
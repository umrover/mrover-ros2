<template>
  <div class="d-flex m-0 p-0 h-100 w-100 gap-2">
    <!-- Left Column: Controls & Creation -->
    <div class="d-flex flex-column w-100">
      <!-- New Waypoint Form -->
      <div class="py-2 border-bottom border-2">
        <div class="d-flex align-items-center justify-content-between mb-2">
          <h4 class="component-header m-0">Add Waypoint</h4>
        </div>
        <div class="d-flex flex-column gap-2">
          <div class="d-flex align-items-center gap-2">
            <label for="waypointname" class="cmd-data-label m-0">Name:</label>
            <input class="form-control cmd-input flex-grow-1" id="waypointname" v-model="name" />
          </div>
          <div class="d-flex gap-2">
            <div class="flex-fill input-group input-group-sm">
              <input class="form-control cmd-input" id="deg1" v-model.number="input.lat.d" />
              <span class="input-group-text">ºN</span>
            </div>
            <div class="flex-fill input-group input-group-sm">
              <input class="form-control cmd-input" id="deg2" v-model.number="input.lon.d" />
              <span class="input-group-text">ºW</span>
            </div>
          </div>
          <button class="btn btn-success btn-sm border-2" @click="addWaypoint(input, false)">
            Add Waypoint
          </button>
        </div>
      </div>

      <!-- Rover Controls -->
      <div class="py-2 border-bottom border-2">
        <div class="d-flex align-items-center justify-content-between mb-2">
          <h4 class="component-header m-0">Rover</h4>
          <button
            v-if="!isRecordingRover"
            class="btn btn-success btn-sm border-2"
            @click="startRecording(false)"
          >
            Start Recording
          </button>
          <button
            v-if="isRecordingRover"
            class="btn btn-danger btn-sm border-2"
            @click="stopRecording"
          >
            Stop Recording
          </button>
        </div>
        <button
          class="btn btn-success btn-sm border-2 w-100"
          @click="addWaypoint(formatted_odom, false)"
        >
          Drop Waypoint at Rover
        </button>
      </div>

      <!-- Drone Controls (es only) -->
      <div v-if="enableDrone" class="py-2 border-bottom border-2">
        <div class="d-flex align-items-center justify-content-between mb-2">
          <h4 class="component-header m-0">Drone</h4>
          <button
            v-if="!isRecordingDrone"
            class="btn btn-success btn-sm border-2"
            @click="startRecording(true)"
          >
            Start Recording
          </button>
          <button
            v-if="isRecordingDrone"
            class="btn btn-danger btn-sm border-2"
            @click="stopRecording"
          >
            Stop Recording
          </button>
        </div>
        <button class="btn btn-info btn-sm border-2 w-100" @click="addWaypoint(input, true)">
          Add Drone Position
        </button>
      </div>

      <!-- Action Buttons -->
      <div class="py-2">
        <button class="btn btn-success btn-sm border-2 w-100 mb-2" @click="showRecordingsModal = true">
          View Recordings
        </button>
        <div class="d-flex gap-2 w-100">
          <button class="btn btn-danger btn-sm border-2" @click="openClearWaypointsModal">
            Clear Waypoints
          </button>
          <button class="btn btn-danger btn-sm border-2" @click="openClearRecordingsModal">
            Clear Recordings
          </button>
        </div>
      </div>
    </div>

    <div class="d-flex flex-column w-100">
      <div class="p-1 mb-2 border-bottom border-2 d-flex justify-content-between align-items-center">
        <h4 class="component-header m-0">Current Course</h4>
        <button class="btn btn-danger btn-sm border-2" @click="clearWaypoint">Clear</button>
      </div>
      <div class="bg-theme-view p-2 rounded overflow-y-auto d-flex flex-column gap-2 flex-grow-1">
        <WaypointItem
          v-for="(waypoint, i) in storedWaypoints"
          :key="i"
          :waypoint="waypoint"
          :index="i"
          @delete="deleteItem($event)"
          @find="findWaypoint($event)"
          @search="searchForWaypoint($event)"
        />
      </div>
    </div>

    <RecordingsModal
      :show="showRecordingsModal"
      @close="showRecordingsModal = false"
    />

    <!-- Clear Waypoints Confirmation Modal -->
    <Teleport to="body">
      <div class="modal fade" id="clearWaypointsModal" tabindex="-1" role="dialog">
        <div class="modal-dialog modal-dialog-centered" role="document">
          <div class="modal-content">
            <div class="modal-header">
              <h5 class="modal-title">Clear Waypoints</h5>
              <button type="button" class="btn-close" @click="closeClearWaypointsModal"></button>
            </div>
            <div class="modal-body">
              <p class="mb-0">Are you sure you want to delete all waypoints? This cannot be undone.</p>
            </div>
            <div class="modal-footer">
              <button type="button" class="btn btn-secondary btn-sm border-2" @click="closeClearWaypointsModal">Cancel</button>
              <button type="button" class="btn btn-danger btn-sm border-2" @click="confirmClearWaypoints">Clear</button>
            </div>
          </div>
        </div>
      </div>

      <!-- Clear Recordings Confirmation Modal -->
      <div class="modal fade" id="clearRecordingsModal" tabindex="-1" role="dialog">
        <div class="modal-dialog modal-dialog-centered" role="document">
          <div class="modal-content">
            <div class="modal-header">
              <h5 class="modal-title">Clear Recordings</h5>
              <button type="button" class="btn-close" @click="closeClearRecordingsModal"></button>
            </div>
            <div class="modal-body">
              <p class="mb-0">Are you sure you want to delete all recordings? This cannot be undone.</p>
            </div>
            <div class="modal-footer">
              <button type="button" class="btn btn-secondary btn-sm border-2" @click="closeClearRecordingsModal">Cancel</button>
              <button type="button" class="btn btn-danger btn-sm border-2" @click="confirmClearRecordings">Clear</button>
            </div>
          </div>
        </div>
      </div>
    </Teleport>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch, onMounted } from 'vue'
import { Modal } from 'bootstrap'
import WaypointItem from './BasicWaypointItem.vue'
import Vuex from 'vuex'
const { mapMutations, mapGetters, mapActions, mapState } = Vuex
import L from 'leaflet'
import type { WebSocketState } from '../types/websocket.js'

export default {
  props: {
    odom: {
      type: Object,
      default: () => ({ latitude_deg: 0, longitude_deg: 0, bearing_deg: 0 }),
    },
    droneWaypointButton: {
      type: Boolean,
      required: false,
    },
  },

  data() {
    return {
      name: 'Waypoint',
      odom_format_in: 'DM',
      input: {
        lat: {
          d: 0,
          m: 0,
          s: 0,
        },
        lon: {
          d: 0,
          m: 0,
          s: 0,
        },
      },

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const rover_latitude_deg = ref(0)
const rover_longitude_deg = ref(0)

const name = ref('Waypoint')
const input = ref({
  lat: { d: 0 },
  lon: { d: 0 },
})
const storedWaypoints = ref<StoreWaypoint[]>([])

const isRecordingRover = ref(false)
const isRecordingDrone = ref(false)
const currentRecordingId = ref<number | null>(null)
const showRecordingsModal = ref(false)

const clearWaypointsModal = ref<Modal | null>(null)
const clearRecordingsModal = ref<Modal | null>(null)

const formatted_odom = computed(() => {
  return {
    lat: { d: rover_latitude_deg.value },
    lon: { d: rover_longitude_deg.value },
  }
})

const navMessage = computed(() => messages.value['nav'])

watch(navMessage, async msg => {
  if (!msg) return
  const navMsg = msg as NavMessage

  if (navMsg.type === 'gps_fix') {
    rover_latitude_deg.value = navMsg.latitude
    rover_longitude_deg.value = navMsg.longitude
  }
})

// Watch waypoints to sync with Backend
watch(storedWaypoints, async (newList) => {
  setWaypointList(newList) // Update map
  
  try {
    const apiWaypoints: APIBasicWaypoint[] = newList.map(wp => ({
      name: wp.name,
      lat: wp.latLng.lat,
      lon: wp.latLng.lng,
      drone: wp.drone,
    }))
    await waypointsAPI.saveBasic(apiWaypoints)
  } catch (error) {
    console.error('Failed to save waypoints:', error)
  }
}, { deep: true })

// Watch map clicks
watch(clickPoint, newClickPoint => {
  input.value.lat.d = newClickPoint.lat
  input.value.lon.d = newClickPoint.lon
})

onMounted(() => {
  setHighlightedWaypoint(-1)
  setSearchWaypoint(-1)
  setWaypointList([])
  clearWaypointsModal.value = new Modal('#clearWaypointsModal', {})
  clearRecordingsModal.value = new Modal('#clearRecordingsModal', {})
  setTimeout(() => loadWaypoints(), 250)
})

const loadWaypoints = async () => {
  try {
    const data = await waypointsAPI.getBasic()
    if (data.status === 'success' && data.waypoints) {
      storedWaypoints.value = data.waypoints.map((wp: APIBasicWaypoint) => ({
        name: wp.name,
        latLng: L.latLng(wp.lat, wp.lon),
        drone: wp.drone,
      }))
    }
  },

  methods: {
    ...mapActions('websocket', ['sendMessage']),

    ...mapMutations('erd', {
      setWaypointList: 'setWaypointList',
      setHighlightedWaypoint: 'setHighlightedWaypoint',
      setSearchWaypoint: 'setSearchWaypoint',
    }),

    ...mapMutations('map', {
      setOdomFormat: 'setOdomFormat',
    }),

const searchForWaypoint = (payload: { index: number }) => {
  setSearchWaypoint(payload.index === searchWaypoint.value ? -1 : payload.index)
}

const clearWaypoint = () => {
  storedWaypoints.value = []
}

const openClearWaypointsModal = () => {
  clearWaypointsModal.value?.show()
}

const closeClearWaypointsModal = () => {
  clearWaypointsModal.value?.hide()
}

const confirmClearWaypoints = async () => {
  try {
    await waypointsAPI.deleteAll()
    storedWaypoints.value = []
  } catch (error) {
    console.error('Failed to clear waypoints:', error)
  } finally {
    closeClearWaypointsModal()
  }
}

const openClearRecordingsModal = () => {
  clearRecordingsModal.value?.show()
}

const closeClearRecordingsModal = () => {
  clearRecordingsModal.value?.hide()
}

const confirmClearRecordings = async () => {
  try {
    await recordingAPI.deleteAll()
  } catch (error) {
    console.error('Failed to clear recordings:', error)
  } finally {
    closeClearRecordingsModal()
  }
}

const startRecording = async (isDrone: boolean) => {
  try {
    const recordingName = `${isDrone ? 'Drone' : 'Rover'} Recording ${new Date().toLocaleString()}`
    const response = await recordingAPI.create(recordingName, isDrone)

    if (response.status === 'success' && response.recording_id) {
      currentRecordingId.value = response.recording_id

      if (isDrone) {
        isRecordingDrone.value = true
      } else {
        isRecordingRover.value = true
      }
      if (this.searchWaypoint == payload.index) {
        this.setSearchWaypoint(-1)
      }
      this.storedWaypoints.splice(payload.index, 1)
    },

    addWaypoint: function (
      coord: {
        lat: { d: number; m: number; s: number }
        lon: { d: number; m: number; s: number }
      },
      isDrone: boolean,
    ) {
      this.storedWaypoints.push({
        name: this.name,
        lat: (coord.lat.d + coord.lat.m / 60 + coord.lat.s / 3600).toFixed(5),
        lon: (coord.lon.d + coord.lon.m / 60 + coord.lon.s / 3600).toFixed(5),
        drone: isDrone,
      })
    },

    findWaypoint: function (payload: { index: number }) {
      if (payload.index === this.highlightedWaypoint) {
        this.setHighlightedWaypoint(-1)
      } else {
        this.setHighlightedWaypoint(payload.index)
      }
    },

    searchForWaypoint: function (payload: { index: number }) {
      if (payload.index === this.searchWaypoint) {
        this.setSearchWaypoint(-1)
      } else {
        this.setSearchWaypoint(payload.index)
      }
    },

    clearWaypoint: function () {
      this.storedWaypoints = []
    },
  },

  watch: {
    storedWaypoints: {
      handler: function (newList) {
        const waypoints = newList.map(
          (waypoint: {
            lat: number
            lon: number
            name: string
            drone: boolean
          }) => {
            return {
              latLng: L.latLng(waypoint.lat, waypoint.lon),
              name: waypoint.name,
              drone: waypoint.drone,
            }
          },
        )
        this.setWaypointList(waypoints)
        this.$store.dispatch('websocket/sendMessage', {
          id: 'waypoints',
          message: {
            type: 'save_basic_waypoint_list',
            data: newList,
          },
        })
      },
      deep: true,
    },

    navMessage: {
      handler: function (msg) {
        if (msg.type == 'get_basic_waypoint_list') {
          this.storedWaypoints = msg.data
          const waypoints = msg.data.map(
            (waypoint: { lat: number; lon: number; name: string }) => {
              const lat = waypoint.lat
              const lon = waypoint.lon
              return { latLng: L.latLng(lat, lon), name: waypoint.name }
            },
          )
          this.setWaypointList(waypoints)
        }
      },
      deep: true,
    },

    odom_format_in: function (newOdomFormat) {
      this.setOdomFormat(newOdomFormat)
      this.input.lat = convertDMS(this.input.lat, newOdomFormat)
      this.input.lon = convertDMS(this.input.lon, newOdomFormat)
    },

    clickPoint: function (newClickPoint) {
      this.input.lat.d = newClickPoint.lat
      this.input.lon.d = newClickPoint.lon
      this.input.lat.m = 0
      this.input.lon.m = 0
      this.input.lat.s = 0
      this.input.lon.s = 0
      this.input.lat = convertDMS(this.input.lat, this.odom_format_in)
      this.input.lon = convertDMS(this.input.lon, this.odom_format_in)
    },
  },

  created: function () {
    this.setHighlightedWaypoint(-1)
    this.setSearchWaypoint(-1)
    this.setWaypointList([])

    this.odom_format_in = this.odom_format

    window.setTimeout(() => {
      this.$store.dispatch('websocket/sendMessage', {
        id: 'waypoints',
        message: {
          type: 'get_basic_waypoint_list',
        },
      })
    }, 250)
  },

  computed: {
    ...mapState('websocket', {
      waypointsMessage: (state: WebSocketState) => state.messages['waypoints'],
    }),
    ...mapGetters('erd', {
      highlightedWaypoint: 'highlightedWaypoint',
      searchWaypoint: 'searchWaypoint',
      clickPoint: 'clickPoint',
    }),

    ...mapGetters('map', {
      odom_format: 'odomFormat',
    }),

    min_enabled: function () {
      return this.odom_format != 'D'
    },

    sec_enabled: function () {
      return this.odom_format == 'DMS'
    },

    formatted_odom: function () {
      return {
        lat: convertDMS(
          { d: this.odom.latitude_deg, m: 0, s: 0 },
          this.odom_format,
        ),
        lon: convertDMS(
          { d: this.odom.longitude_deg, m: 0, s: 0 },
          this.odom_format,
        ),
      }
    },
  },

  components: {
    WaypointItem,
  },
}
</script>

<style scoped>
.input-group-text {
  font-size: 0.75rem;
  min-width: 40px;
  justify-content: center;
}
</style>

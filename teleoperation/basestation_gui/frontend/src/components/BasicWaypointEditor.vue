<template>
  <div class="wrapper d-flex m-0 p-2 justify-content-between gap-3 w-100 h-100">
    <div class="d-flex flex-column w-100 gap-2">
      <div class="d-flex flex-column gap-2 border border-2 rounded p-2">
        <div class="d-flex align-items-center">
          <label for="waypointname" class="form-label m-0 me-2">Name:</label>
          <div class="col">
            <input class="form-control" id="waypointname" v-model="name" />
          </div>
        </div>
        <div class="d-flex gap-2">
          <div class="flex-fill input-group">
            <input
              class="form-control"
              id="deg1"
              v-model.number="input.lat.d"
            />
            <span class="input-group-text font-monospace px-2">ºN</span>
          </div>
          <div class="flex-fill input-group">
            <input
              class="form-control"
              id="deg2"
              v-model.number="input.lon.d"
            />
            <span class="input-group-text font-monospace px-2">ºW</span>
          </div>
        </div>
        <button class="btn btn-success" @click="addWaypoint(input, false)">
          Add Waypoint
        </button>
      </div>
      <div class="border border-2 rounded p-2 gap-2 d-flex flex-column">
        <div class="d-flex justify-content-between align-items-center gap-2">
          <h4>Rover</h4>
          <button
            v-if="!isRecordingRover"
            class="btn btn-success btn-sm"
            @click="startRecording(false)"
          >
            Start Recording
          </button>
          <button
            v-if="isRecordingRover"
            class="btn btn-danger btn-sm"
            @click="stopRecording"
          >
            Stop Recording
          </button>
        </div>
        <button
          class="btn btn-success"
          @click="addWaypoint(formatted_odom, false)"
        >
          Drop Waypoint at Rover
        </button>
      </div>
      <div class="border border-2 rounded d-flex flex-column p-2 gap-2">
        <div class="d-flex justify-content-between align-items-center gap-2">
          <h4>Drone</h4>
          <button
            v-if="enableDrone && !isRecordingDrone"
            class="btn btn-success btn-sm"
            @click="startRecording(true)"
          >
            Start Recording
          </button>
          <button
            v-if="enableDrone && isRecordingDrone"
            class="btn btn-danger btn-sm"
            @click="stopRecording"
          >
            Stop Recording
          </button>
        </div>
        <button
          v-if="enableDrone"
          class="btn btn-info"
          @click="addWaypoint(input, true)"
        >
          Add Drone Position
        </button>
      </div>
      <button class="btn btn-success" @click="showRecordingsModal = true">View Recordings</button>
      <div class="d-flex gap-2">
        <button class="btn btn-danger flex-fill" @click="clearAllWaypoints">Clear Waypoints</button>
        <button class="btn btn-danger flex-fill" @click="clearAllRecordings">Clear Recordings</button>
      </div>
    </div>

    <div class="d-flex flex-column w-100">
      <div class="d-flex mb-2 align-items-center justify-content-between">
        <h4 class="m-0 p-0">Current Course</h4>
        <button class="btn btn-danger" @click="clearWaypoint">Clear</button>
      </div>
      <div class="waypoint-wrapper overflow-y-scroll d-flex flex-column gap-2">
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
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch, onMounted } from 'vue'
import WaypointItem from './BasicWaypointItem.vue'
import RecordingsModal from './RecordingsModal.vue'
import { useErdStore } from '@/stores/erd'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import L from 'leaflet'
import { waypointsAPI, recordingAPI } from '@/utils/api'
import type { StoreWaypoint, APIBasicWaypoint } from '@/types/waypoints'
import type { NavMessage } from '@/types/coordinates'

defineProps({
  enableDrone: {
    type: Boolean,
    required: false,
  },
})

const erdStore = useErdStore()
const { highlightedWaypoint, searchWaypoint, clickPoint } =
  storeToRefs(erdStore)
const { setWaypointList, setHighlightedWaypoint, setSearchWaypoint } = erdStore

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
const recordingSequence = ref(0)
const lastRecordedLat = ref<number | null>(null)
const lastRecordedLon = ref<number | null>(null)
const RECORDING_THRESHOLD = 0.00001

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

    if (isRecordingRover.value && currentRecordingId.value !== null) {
      if (shouldRecordWaypoint(navMsg.latitude, navMsg.longitude)) {
        try {
          await recordingAPI.addWaypoint(
            currentRecordingId.value,
            navMsg.latitude,
            navMsg.longitude,
            recordingSequence.value
          )
          lastRecordedLat.value = navMsg.latitude
          lastRecordedLon.value = navMsg.longitude
          recordingSequence.value++
        } catch (error) {
          console.error('Failed to record waypoint:', error)
        }
      }
    }
  }
})

const saveWaypoints = async (waypoints: StoreWaypoint[]) => {
  try {
    const apiWaypoints: APIBasicWaypoint[] = waypoints.map(wp => ({
      name: wp.name,
      lat: wp.latLng.lat,
      lon: wp.latLng.lng,
      drone: wp.drone,
    }))
    await waypointsAPI.saveBasic(apiWaypoints)
  } catch (error) {
    console.error('Failed to save waypoints:', error)
  }
}

const loadWaypoints = async () => {
  try {
    const data = await waypointsAPI.getBasic()
    if (data.status === 'success' && data.waypoints) {
      storedWaypoints.value = data.waypoints.map((wp: APIBasicWaypoint) => ({
        name: wp.name,
        latLng: L.latLng(wp.lat, wp.lon),
        drone: wp.drone,
      }))
      setWaypointList(storedWaypoints.value)
    }
  } catch (error) {
    console.error('Failed to load waypoints:', error)
  }
}

const deleteItem = (payload: { index: number }) => {
  if (highlightedWaypoint.value == payload.index) {
    setHighlightedWaypoint(-1)
  }
  if (searchWaypoint.value == payload.index) {
    setSearchWaypoint(-1)
  }
  storedWaypoints.value.splice(payload.index, 1)
}

const addWaypoint = (
  coord: {
    lat: { d: number }
    lon: { d: number }
  },
  isDrone: boolean,
) => {
  storedWaypoints.value.push({
    name: name.value,
    latLng: L.latLng(coord.lat.d, coord.lon.d),
    drone: isDrone,
  })
}

const findWaypoint = (payload: { index: number }) => {
  if (payload.index === highlightedWaypoint.value) {
    setHighlightedWaypoint(-1)
  } else {
    setHighlightedWaypoint(payload.index)
  }
}

const searchForWaypoint = (payload: { index: number }) => {
  if (payload.index === searchWaypoint.value) {
    setSearchWaypoint(-1)
  } else {
    setSearchWaypoint(payload.index)
  }
}

const clearWaypoint = () => {
  storedWaypoints.value = []
}

const clearAllWaypoints = async () => {
  if (!confirm('Are you sure you want to delete all waypoints? This cannot be undone.')) {
    return
  }
  try {
    await waypointsAPI.deleteAll()
    storedWaypoints.value = []
  } catch (error) {
    console.error('Failed to clear waypoints:', error)
  }
}

const clearAllRecordings = async () => {
  if (!confirm('Are you sure you want to delete all recordings? This cannot be undone.')) {
    return
  }
  try {
    await recordingAPI.deleteAll()
  } catch (error) {
    console.error('Failed to clear recordings:', error)
  }
}

const startRecording = async (isDrone: boolean) => {
  try {
    const recordingName = `${isDrone ? 'Drone' : 'Rover'} Recording ${new Date().toLocaleString()}`
    const response = await recordingAPI.create(recordingName, isDrone)

    if (response.status === 'success' && response.recording_id) {
      currentRecordingId.value = response.recording_id
      recordingSequence.value = 0
      lastRecordedLat.value = null
      lastRecordedLon.value = null

      if (isDrone) {
        isRecordingDrone.value = true
      } else {
        isRecordingRover.value = true
      }

      console.log(`Started recording: ${recordingName}`)
    } else {
      console.error('Failed to start recording:', response.message)
    }
  } catch (error) {
    console.error('Error starting recording:', error)
  }
}

const stopRecording = async () => {
  if (currentRecordingId.value) {
    console.log(`Stopped recording ID: ${currentRecordingId.value} with ${recordingSequence.value} waypoints`)
  }

  currentRecordingId.value = null
  recordingSequence.value = 0
  lastRecordedLat.value = null
  lastRecordedLon.value = null
  isRecordingRover.value = false
  isRecordingDrone.value = false
}

const shouldRecordWaypoint = (lat: number, lon: number): boolean => {
  if (lastRecordedLat.value === null || lastRecordedLon.value === null) {
    return true
  }

  const latDiff = Math.abs(lat - lastRecordedLat.value)
  const lonDiff = Math.abs(lon - lastRecordedLon.value)
  const distance = Math.sqrt(latDiff * latDiff + lonDiff * lonDiff)

  return distance >= RECORDING_THRESHOLD
}

watch(
  storedWaypoints,
  newList => {
    setWaypointList(newList)
    saveWaypoints(newList)
  },
  { deep: true },
)

watch(clickPoint, newClickPoint => {
  input.value.lat.d = newClickPoint.lat
  input.value.lon.d = newClickPoint.lon
})

onMounted(() => {
  setHighlightedWaypoint(-1)
  setSearchWaypoint(-1)
  setWaypointList([])

  setTimeout(() => {
    loadWaypoints()
  }, 250)
})
</script>

<style scoped>
.waypoint-wrapper {
  flex: 1;
  overflow-y: auto;
  background-color: #dddddd;
  padding: 8px;
  border-radius: 8px;
}

.waypoint-col {
  min-width: 300px;
}

.label-width {
  min-width: 40px;
}
</style>

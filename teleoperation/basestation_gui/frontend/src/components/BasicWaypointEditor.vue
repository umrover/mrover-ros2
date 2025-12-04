<template>
  <div class="wrapper d-flex m-0 p-2 justify-content-between gap-3 w-100 h-100">
    <!-- Left Column: Controls & Creation -->
    <div class="d-flex flex-column w-100 gap-2">
      <!-- New Waypoint Form -->
      <div class="d-flex flex-column gap-2 border border-2 rounded p-2">
        <div class="d-flex align-items-center">
          <label for="waypointname" class="form-label m-0 me-2">Name:</label>
          <div class="col">
            <input class="form-control" id="waypointname" v-model="name" />
          </div>
        </div>
        <div class="d-flex gap-2">
          <div class="flex-fill input-group">
            <input class="form-control" id="deg1" v-model.number="input.lat.d" />
            <span class="input-group-text font-monospace px-2">ºN</span>
          </div>
          <div class="flex-fill input-group">
            <input class="form-control" id="deg2" v-model.number="input.lon.d" />
            <span class="input-group-text font-monospace px-2">ºW</span>
          </div>
        </div>
        <button class="btn btn-success" @click="addWaypoint(input, false)">
          Add Waypoint
        </button>
      </div>

      <!-- Rover Controls -->
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

      <!-- Drone Controls (Conditional) -->
      <div v-if="enableDrone" class="border border-2 rounded d-flex flex-column p-2 gap-2">
        <div class="d-flex justify-content-between align-items-center gap-2">
          <h4>Drone</h4>
          <button
            v-if="!isRecordingDrone"
            class="btn btn-success btn-sm"
            @click="startRecording(true)"
          >
            Start Recording
          </button>
          <button
            v-if="isRecordingDrone"
            class="btn btn-danger btn-sm"
            @click="stopRecording"
          >
            Stop Recording
          </button>
        </div>
        <button class="btn btn-info" @click="addWaypoint(input, true)">
          Add Drone Position
        </button>
      </div>

      <!-- Action Buttons -->
      <button class="btn btn-success" @click="showRecordingsModal = true">
        View Recordings
      </button>
      <div class="d-flex gap-2">
        <button class="btn btn-danger flex-fill" @click="clearAllWaypoints">
          Clear Waypoints
        </button>
        <button class="btn btn-danger flex-fill" @click="clearAllRecordings">
          Clear Recordings
        </button>
      </div>
    </div>

    <!-- Right Column: Active Route List -->
    <div class="d-flex flex-column w-100">
      <div class="d-flex mb-2 align-items-center justify-content-between">
        <h4 class="m-0 p-0">Current Course</h4>
        <button class="btn btn-danger" @click="clearWaypoint">Clear</button>
      </div>
      <div class="waypoint-wrapper overflow-y-scroll d-flex flex-column gap-2 flex-grow-1">
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
const { highlightedWaypoint, searchWaypoint, clickPoint } = storeToRefs(erdStore)
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
  } catch (error) {
    console.error('Failed to load waypoints:', error)
  }
}

const addWaypoint = (coord: { lat: { d: number }, lon: { d: number } }, isDrone: boolean) => {
  storedWaypoints.value.push({
    name: name.value,
    latLng: L.latLng(coord.lat.d, coord.lon.d),
    drone: isDrone,
  })
}

const deleteItem = (payload: { index: number }) => {
  if (highlightedWaypoint.value == payload.index) setHighlightedWaypoint(-1)
  if (searchWaypoint.value == payload.index) setSearchWaypoint(-1)
  storedWaypoints.value.splice(payload.index, 1)
}

const findWaypoint = (payload: { index: number }) => {
  setHighlightedWaypoint(payload.index === highlightedWaypoint.value ? -1 : payload.index)
}

const searchForWaypoint = (payload: { index: number }) => {
  setSearchWaypoint(payload.index === searchWaypoint.value ? -1 : payload.index)
}

const clearWaypoint = () => {
  storedWaypoints.value = []
}

const clearAllWaypoints = async () => {
  if (!confirm('Are you sure you want to delete all waypoints? This cannot be undone.')) return
  try {
    await waypointsAPI.deleteAll()
    storedWaypoints.value = []
  } catch (error) {
    console.error('Failed to clear waypoints:', error)
  }
}

const clearAllRecordings = async () => {
  if (!confirm('Are you sure you want to delete all recordings? This cannot be undone.')) return
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

      if (isDrone) {
        isRecordingDrone.value = true
      } else {
        isRecordingRover.value = true
      }

      console.log(`Recording started: ${recordingName} (ID: ${response.recording_id})`)
    } else {
      console.error('Failed to start recording:', response.message)
    }
  } catch (error) {
    console.error('Error starting recording:', error)
  }
}

const stopRecording = async () => {
  try {
    const response = await recordingAPI.stop()

    if (response.status === 'success') {
      console.log(`Recording stopped`)
    } else {
      console.error('Failed to stop recording:', response.message)
    }
  } catch (error) {
    console.error('Error stopping recording:', error)
  } finally {
    isRecordingRover.value = false
    isRecordingDrone.value = false
    currentRecordingId.value = null
  }
}
</script>

<style scoped>
.waypoint-wrapper {
  background-color: #dddddd;
  padding: 8px;
  border-radius: 8px;
}
</style>
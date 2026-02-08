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
            <input class="form-control cmd-input flex-grow-1" id="waypointname" data-testid="pw-basic-wp-name" v-model="name" />
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
          <button class="btn btn-success btn-sm border-2" data-testid="pw-basic-wp-add-btn" @click="addWaypoint(input, false)">
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
        <button class="btn btn-success btn-sm border-2 w-100 mb-2" data-testid="pw-basic-wp-recordings-btn" @click="showRecordingsModal = true">
          View Recordings
        </button>
        <div class="d-flex gap-2 w-100">
          <button class="btn btn-danger btn-sm border-2" data-testid="pw-basic-wp-clear-btn" @click="openClearWaypointsModal">
            Clear Waypoints
          </button>
          <button class="btn btn-danger btn-sm border-2" @click="openClearRecordingsModal">
            Clear Recordings
          </button>
        </div>
      </div>
    </div>

    <!-- Right Column: Active Route List -->
    <div class="d-flex flex-column w-100">
      <div class="p-1 mb-2 border-bottom border-2 d-flex justify-content-between align-items-center">
        <h4 class="component-header m-0">Current Course</h4>
        <button class="btn btn-danger btn-sm border-2" @click="clearWaypoint">Clear</button>
      </div>
      <div class="bg-theme-view p-2 rounded overflow-y-auto d-flex flex-column gap-2 flex-grow-1" data-testid="pw-basic-wp-list">
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
  erdStore.waypointList = newList // Update map
  
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
  erdStore.highlightedWaypoint = -1
  erdStore.searchWaypoint = -1
  erdStore.waypointList = []
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
  if (highlightedWaypoint.value == payload.index) erdStore.highlightedWaypoint = -1
  if (searchWaypoint.value == payload.index) erdStore.searchWaypoint = -1
  storedWaypoints.value.splice(payload.index, 1)
}

const findWaypoint = (payload: { index: number }) => {
  erdStore.highlightedWaypoint = payload.index === highlightedWaypoint.value ? -1 : payload.index
}

const searchForWaypoint = (payload: { index: number }) => {
  erdStore.searchWaypoint = payload.index === searchWaypoint.value ? -1 : payload.index
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
.input-group-text {
  font-size: 0.75rem;
  min-width: 40px;
  justify-content: center;
}
</style>

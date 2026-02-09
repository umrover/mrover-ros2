<template>
  <div class="d-flex m-0 p-0 h-100 w-100 gap-2">
    <div class="d-flex flex-column w-100">
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
              <span class="input-group-text">N</span>
            </div>
            <div class="flex-fill input-group input-group-sm">
              <input class="form-control cmd-input" id="deg2" v-model.number="input.lon.d" />
              <span class="input-group-text">W</span>
            </div>
          </div>
          <button class="btn btn-success btn-sm border-2" data-testid="pw-basic-wp-add-btn" @click="handleAddWaypoint(input, false)">
            Add Waypoint
          </button>
        </div>
      </div>

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
          @click="handleAddWaypoint(formatted_odom, false)"
        >
          Drop Waypoint at Rover
        </button>
      </div>

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
        <button class="btn btn-info btn-sm border-2 w-100" @click="handleAddWaypoint(input, true)">
          Add Drone Position
        </button>
      </div>

      <div class="py-2">
        <button class="btn btn-success btn-sm border-2 w-100 mb-2" data-testid="pw-basic-wp-recordings-btn" @click="showRecordingsModal = true">
          View Recordings
        </button>
        <div class="d-flex gap-2 w-100">
          <button class="btn btn-danger btn-sm border-2" data-testid="pw-basic-wp-clear-btn" @click="clearWaypointsModal?.open()">
            Clear Waypoints
          </button>
          <button class="btn btn-danger btn-sm border-2" @click="clearRecordingsModal?.open()">
            Clear Recordings
          </button>
        </div>
      </div>
    </div>

    <div class="d-flex flex-column w-100">
      <div class="p-1 mb-2 border-bottom border-2 d-flex justify-content-between align-items-center">
        <h4 class="component-header m-0">Current Course</h4>
        <button class="btn btn-danger btn-sm border-2" @click="handleClearList">Clear</button>
      </div>
      <div class="bg-theme-view p-2 rounded overflow-y-auto d-flex flex-column gap-2 flex-grow-1" data-testid="pw-basic-wp-list">
        <WaypointItem
          v-for="(waypoint, i) in erdStore.waypoints"
          :key="waypoint.db_id || i"
          :waypoint="waypoint"
          :index="i"
          @delete="handleDelete($event)"
          @find="erdStore.setHighlighted($event.index)"
          @search="erdStore.setSearch($event.index)"
        />
      </div>
    </div>

    <RecordingsModal
      :show="showRecordingsModal"
      @close="showRecordingsModal = false"
    />

    <ConfirmModal
      ref="clearWaypointsModal"
      modal-id="clearWaypointsModal"
      title="Clear Waypoints"
      message="Are you sure you want to delete all waypoints? This cannot be undone."
      confirm-text="Clear"
      @confirm="handleConfirmClearWaypoints"
    />

    <ConfirmModal
      ref="clearRecordingsModal"
      modal-id="clearRecordingsModal"
      title="Clear Recordings"
      message="Are you sure you want to delete all recordings? This cannot be undone."
      confirm-text="Clear"
      @confirm="handleConfirmClearRecordings"
    />
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch, onMounted } from 'vue'
import WaypointItem from './BasicWaypointItem.vue'
import RecordingsModal from './RecordingsModal.vue'
import ConfirmModal from './ConfirmModal.vue'
import { useErdStore } from '@/stores/erd'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import { recordingAPI } from '@/utils/api'
import type { NavMessage } from '@/types/coordinates'

defineProps({
  enableDrone: {
    type: Boolean,
    required: false,
  },
})

const erdStore = useErdStore()
const { clickPoint } = storeToRefs(erdStore)

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const rover_latitude_deg = ref(0)
const rover_longitude_deg = ref(0)

const name = ref('Waypoint')
const input = ref({
  lat: { d: 0 },
  lon: { d: 0 },
})

const isRecordingRover = ref(false)
const isRecordingDrone = ref(false)
const currentRecordingId = ref<number | null>(null)
const showRecordingsModal = ref(false)

const clearWaypointsModal = ref<InstanceType<typeof ConfirmModal> | null>(null)
const clearRecordingsModal = ref<InstanceType<typeof ConfirmModal> | null>(null)

const formatted_odom = computed(() => ({
  lat: { d: rover_latitude_deg.value },
  lon: { d: rover_longitude_deg.value },
}))

const navMessage = computed(() => messages.value['nav'])

watch(navMessage, (msg) => {
  if (!msg) return
  const navMsg = msg as NavMessage
  if (navMsg.type === 'gps_fix') {
    rover_latitude_deg.value = navMsg.latitude
    rover_longitude_deg.value = navMsg.longitude
  }
})

watch(clickPoint, (pt) => {
  input.value.lat.d = pt.lat
  input.value.lon.d = pt.lon
})

onMounted(() => {
  erdStore.highlightedWaypoint = -1
  erdStore.searchWaypoint = -1
  erdStore.fetchAll()
})

async function handleAddWaypoint(coord: { lat: { d: number }; lon: { d: number } }, isDrone: boolean) {
  try {
    await erdStore.addWaypoint({
      name: name.value,
      lat: coord.lat.d,
      lon: coord.lon.d,
      drone: isDrone,
    })
  } catch (error) {
    console.error('Failed to add waypoint:', error)
  }
}

function handleDelete(payload: { index: number }) {
  erdStore.deleteWaypoint(payload.index)
}

function handleClearList() {
  erdStore.waypoints = []
}

async function handleConfirmClearWaypoints() {
  try {
    await erdStore.clearAll()
  } catch (error) {
    console.error('Failed to clear waypoints:', error)
  }
}

async function handleConfirmClearRecordings() {
  try {
    await recordingAPI.deleteAll()
  } catch (error) {
    console.error('Failed to clear recordings:', error)
  }
}

async function startRecording(isDrone: boolean) {
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
    }
  } catch (error) {
    console.error('Error starting recording:', error)
  }
}

async function stopRecording() {
  try {
    await recordingAPI.stop()
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

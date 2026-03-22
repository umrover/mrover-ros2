<template>
  <div class="flex m-0 p-0 h-full w-full gap-2">
    <div class="flex flex-col w-full">
      <div class="py-2 border-b-2">
        <div class="flex items-center justify-between mb-2">
          <h4 class="component-header">Add Waypoint</h4>
        </div>
        <div class="flex flex-col gap-2">
          <div class="flex items-center gap-2">
            <label for="waypointname" class="data-label m-0">Name:</label>
            <input class="form-control input grow" id="waypointname" data-testid="pw-basic-wp-name" v-model="name" />
          </div>
          <div class="flex gap-2">
            <div class="flex-1 input-group input-group-sm">
              <input class="form-control input" id="deg1" v-model.number="input.lat.d" />
              <span class="input-group-text">N</span>
            </div>
            <div class="flex-1 input-group input-group-sm">
              <input class="form-control input" id="deg2" v-model.number="input.lon.d" />
              <span class="input-group-text">W</span>
            </div>
          </div>
          <button class="btn btn-success btn-sm" data-testid="pw-basic-wp-add-btn" @click="handleAddWaypoint(input, false)">
            Add Waypoint
          </button>
        </div>
      </div>

      <div class="py-2 border-b-2">
        <div class="flex items-center justify-between mb-2">
          <h4 class="component-header">Rover</h4>
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
          class="btn btn-success btn-sm w-full"
          @click="handleAddWaypoint(formatted_odom, false)"
        >
          Drop Waypoint at Rover
        </button>
      </div>

      <div v-if="enableDrone" class="py-2 border-b-2">
        <div class="flex items-center justify-between mb-2">
          <h4 class="component-header">Drone</h4>
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
        <button class="btn btn-info btn-sm w-full" @click="handleAddWaypoint(input, true)">
          Add Drone Position
        </button>
      </div>

      <div class="py-2">
        <button class="btn btn-success btn-sm w-full mb-2" data-testid="pw-basic-wp-recordings-btn" @click="showRecordingsModal = true">
          View Recordings
        </button>
        <div class="flex gap-2 w-full">
          <button class="btn btn-danger btn-sm" data-testid="pw-basic-wp-clear-btn" @click="clearWaypointsModal?.open()">
            Clear Waypoints
          </button>
          <button class="btn btn-danger btn-sm" @click="clearRecordingsModal?.open()">
            Clear Recordings
          </button>
        </div>
      </div>
    </div>

    <div class="flex flex-col w-full">
      <div class="p-1 mb-2 border-b-2 flex justify-between items-center">
        <h4 class="component-header">Current Course</h4>
        <button class="btn btn-danger btn-sm" @click="handleClearList">Clear</button>
      </div>
      <VueDraggable
        v-model="erdStore.waypoints"
        handle=".drag-handle"
        ghost-class="drag-ghost"
        class="bg-theme-view p-2 rounded overflow-y-auto flex flex-col gap-2 grow relative"
        data-testid="pw-basic-wp-list"
      >
        <div v-if="erdStore.waypoints.length === 0" class="course-empty-state">
          <i class="bi bi-signpost-split"></i>
          <span>No waypoints in course</span>
        </div>
        <WaypointItem
          v-for="(waypoint, i) in erdStore.waypoints"
          :key="waypoint.db_id || i"
          :waypoint="waypoint"
          :index="i"
          @delete="handleDelete($event)"
          @find="erdStore.setHighlighted($event.index)"
          @search="erdStore.setSearch($event.index)"
        />
      </VueDraggable>
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
import { VueDraggable } from 'vue-draggable-plus'
import WaypointItem from './BasicWaypointItem.vue'
import RecordingsModal from './RecordingsModal.vue'
import ConfirmModal from './ConfirmModal.vue'
import { useErdStore } from '@/stores/erd'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import { recordingAPI } from '@/utils/api'
import type { GpsFixMessage } from '@/types/coordinates'

defineProps({
  enableDrone: {
    type: Boolean,
    required: false,
  },
})

const erdStore = useErdStore()
const { clickPoint } = storeToRefs(erdStore)

const websocketStore = useWebsocketStore()

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

websocketStore.onMessage<GpsFixMessage>('nav', 'gps_fix', (msg) => {
  rover_latitude_deg.value = msg.latitude
  rover_longitude_deg.value = msg.longitude
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
  justify-content: center;
  min-width: 40px;
  font-size: 0.75rem;
}
</style>

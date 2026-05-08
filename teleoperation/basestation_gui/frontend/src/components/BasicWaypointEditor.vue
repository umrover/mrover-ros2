<template>
  <div class="flex m-0 p-0 h-full w-full gap-6">
    <div class="flex flex-col w-full gap-4">
      <div class="flex flex-col gap-2">
        <div class="p-1 border-b-2 flex justify-between items-center h-[var(--btn-height-md)]">
          <h4 class="component-header">Course Planning</h4>
          <div class="h-[var(--btn-height-sm)]"></div> <!-- Spacer to match button height -->
        </div>
        <div class="grid grid-cols-1 gap-2">
          <button 
            class="btn btn-success btn-sm w-full" 
            @click="handleAddWaypoint(formatted_odom, false)"
          >
            Drop Waypoint at Rover
          </button>
          <button 
            v-if="enableDrone" 
            class="btn btn-info btn-sm w-full" 
            @click="handleAddWaypoint(input, true)"
          >
            Add Drone Position
          </button>
        </div>

        <div class="flex flex-col gap-3 p-3 border rounded">
          <div class="flex items-center gap-2">
            <label for="waypointname" class="data-label m-0">Name:</label>
            <input class="form-control form-control-sm grow" id="waypointname" data-testid="pw-basic-wp-name" v-model="name" />
          </div>
          <div class="flex gap-2">
            <div class="flex-1 input-group input-group-sm">
              <input class="form-control form-control-sm" id="deg1" v-model.number="input.lat.d" />
              <span class="input-group-text">N</span>
            </div>
            <div class="flex-1 input-group input-group-sm">
              <input class="form-control form-control-sm" id="deg2" v-model.number="input.lon.d" />
              <span class="input-group-text">W</span>
            </div>
          </div>
          <button class="btn btn-success btn-sm" data-testid="pw-basic-wp-add-btn" @click="handleAddWaypoint(input, false)">
            Add Waypoint
          </button>
        </div>
      </div>

      <div class="flex flex-col gap-2">
        <div class="p-1 border-b-2 flex justify-between items-center h-[var(--btn-height-md)]">
          <h4 class="component-header">Path Recording</h4>
          <div class="flex items-center h-[var(--btn-height-sm)]">
            <div v-if="isRecordingRover || isRecordingDrone" class="recording-badge flex items-center gap-1">
              <span class="recording-dot"></span>
              REC
            </div>
          </div>
        </div>

        <div class="grid grid-cols-1 gap-3">
          <div class="flex items-center justify-between p-2 border rounded">
            <span class="data-label">Rover Telemetry</span>
            <button
              v-if="!isRecordingRover"
              class="btn btn-success btn-sm w-20"
              @click="startRecording(false)"
              :disabled="isRecordingDrone"
            >
              Start
            </button>
            <button v-else class="btn btn-danger btn-sm w-20" @click="stopRecording">Stop</button>
          </div>

          <div v-if="enableDrone" class="flex items-center justify-between p-2 border rounded">
            <span class="data-label">Drone Telemetry</span>
            <button
              v-if="!isRecordingDrone"
              class="btn btn-success btn-sm w-20"
              @click="startRecording(true)"
              :disabled="isRecordingRover"
            >
              Start
            </button>
            <button v-else class="btn btn-danger btn-sm w-20" @click="stopRecording">Stop</button>
          </div>
        </div>

        <button
          class="btn btn-primary btn-sm w-full"
          data-testid="pw-basic-wp-recordings-btn"
          @click="showRecordingsModal = true"
        >
          View Recordings
        </button>
      </div>
    </div>

    <div class="flex flex-col w-full gap-2">
      <div class="p-1 border-b-2 flex justify-between items-center h-[var(--btn-height-md)]">
        <h4 class="component-header">Current Course</h4>
        <button class="btn btn-danger btn-sm" data-testid="pw-basic-wp-clear-btn" @click="clearWaypointsModal?.open()">Clear</button>
      </div>
      <VueDraggable
        v-model="erdStore.waypoints"
        handle=".drag-handle"
        ghost-class="drag-ghost"
        class="bg-theme-view p-2 rounded overflow-y-auto flex flex-col gap-2 grow relative border"
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

async function handleConfirmClearWaypoints() {
  try {
    await erdStore.clearAll()
  } catch (error) {
    console.error('Failed to clear waypoints:', error)
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

<script lang="ts">
export default {
  name: 'BasicWaypointEditor',
}
</script>

<style scoped>
.input-group-text {
  justify-content: center;
  min-width: 40px;
  font-size: 0.75rem;
}

.recording-badge {
  font-size: 0.625rem;
  font-weight: 800;
  color: var(--status-error);
  padding: 0.125rem 0.375rem;
  background-color: rgba(var(--status-error-rgb), 0.1);
  border: 1px solid var(--status-error);
  border-radius: var(--radius-sm);
  letter-spacing: 0.05em;
}

.recording-dot {
  width: 6px;
  height: 6px;
  background-color: var(--status-error);
  border-radius: 50%;
  animation: pulse 1.5s infinite;
}

@keyframes pulse {
  0% { opacity: 1; transform: scale(1); }
  50% { opacity: 0.4; transform: scale(1.2); }
  100% { opacity: 1; transform: scale(1); }
}
</style>

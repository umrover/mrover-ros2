<template>
  <div class="flex m-0 p-0 h-full w-full gap-6">
    <div class="flex flex-col w-full gap-4">
      <div class="flex flex-col gap-2">
        <div class="p-1 border-b-2 flex justify-between items-center h-[var(--btn-height-md)]">
          <h4 class="component-header">Course Planning</h4>
          <div class="h-[var(--btn-height-sm)]"></div>
        </div>
        <div class="grid grid-cols-1 gap-2">
          <button
            class="btn btn-success btn-sm w-full"
            @click="handleDropWaypointAtRover"
          >
            Drop Waypoint at Rover
          </button>
        </div>

        <div class="flex flex-col gap-2 p-3 border rounded">
          <div class="input-group input-group-sm">
            <span class="input-group-text field-label">Name</span>
            <input class="form-control form-control-sm" id="sciencewpname" v-model="name" />
          </div>
          <div class="input-group input-group-sm">
            <span class="input-group-text field-label">Lat</span>
            <input class="form-control form-control-sm" id="sci-deg1" v-model.number="input.lat.d" placeholder="required" />
            <span class="input-group-text">N</span>
          </div>
          <div class="input-group input-group-sm">
            <span class="input-group-text field-label">Lon</span>
            <input class="form-control form-control-sm" id="sci-deg2" v-model.number="input.lon.d" placeholder="required" />
            <span class="input-group-text">W</span>
          </div>
          <div class="input-group input-group-sm">
            <span class="input-group-text field-label">Alt</span>
            <input class="form-control form-control-sm" id="sci-alt" v-model.number="input.alt" placeholder="required" />
            <span class="input-group-text">m</span>
          </div>
          <button class="btn btn-success btn-sm" :disabled="!canAddWaypoint" @click="handleAddWaypoint(input)">
            Add Waypoint
          </button>
        </div>
      </div>

      <div class="flex flex-col gap-2">
        <div class="p-1 border-b-2 flex justify-between items-center h-[var(--btn-height-md)]">
          <h4 class="component-header">Export Course</h4>
          <div class="h-[var(--btn-height-sm)]"></div>
        </div>
        <div class="grid grid-cols-2 gap-2">
          <button
            class="btn btn-info btn-sm"
            :disabled="scienceStore.waypoints.length === 0"
            @click="scienceStore.exportToText()"
          >
            <i class="bi bi-download"></i> TXT
          </button>
          <button
            class="btn btn-primary btn-sm"
            @click="showCourseMapModal = true"
          >
            <i class="bi bi-map"></i> Map
          </button>
        </div>
      </div>

      <div class="flex flex-col gap-2">
        <div class="p-1 border-b-2 flex justify-between items-center h-[var(--btn-height-md)]">
          <h4 class="component-header">Path Recording</h4>
          <div class="flex items-center h-[var(--btn-height-sm)]">
            <div v-if="isRecording" class="recording-badge flex items-center gap-1">
              <span class="recording-dot"></span>
              REC
            </div>
          </div>
        </div>
        <div class="flex items-center justify-between p-2 border rounded">
          <span class="data-label">Rover Telemetry</span>
          <button
            v-if="!isRecording"
            class="btn btn-success btn-sm w-20"
            @click="startRecording"
          >
            Start
          </button>
          <button v-else class="btn btn-danger btn-sm w-20" @click="stopRecording">Stop</button>
        </div>
      </div>
    </div>

    <div class="flex flex-col w-full gap-2">
      <div class="p-1 border-b-2 flex justify-between items-center h-[var(--btn-height-md)]">
        <h4 class="component-header">Current Course</h4>
        <button class="btn btn-danger btn-sm btn-icon" title="Reset table" @click="resetTableModal?.open()">
          <i class="bi bi-arrow-counterclockwise"></i>
        </button>
      </div>
      <VueDraggable
        v-model="scienceStore.waypoints"
        handle=".drag-handle"
        ghost-class="drag-ghost"
        class="bg-theme-view p-2 rounded overflow-y-auto flex flex-col gap-2 grow relative border"
      >
        <div v-if="scienceStore.waypoints.length === 0" class="course-empty-state">
          <i class="bi bi-signpost-split"></i>
          <span>No waypoints in course</span>
        </div>
        <WaypointItem
          v-for="(waypoint, i) in scienceStore.waypoints"
          :key="waypoint.id || i"
          :waypoint="waypoint"
          :index="i"
          @delete="handleDelete($event)"
          @find="scienceStore.setHighlighted($event.index)"
          @search="scienceStore.setSearch($event.index)"
        />
      </VueDraggable>
    </div>

    <CourseMapModal
      :show="showCourseMapModal"
      @close="showCourseMapModal = false"
    />

    <ConfirmModal
      ref="resetTableModal"
      modal-id="scienceResetTableModal"
      title="Reset Waypoints Table"
      message="This will drop and recreate the science_waypoints table. All data will be lost."
      confirm-text="Reset"
      @confirm="handleConfirmResetTable"
    />
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch, onMounted } from 'vue'
import { VueDraggable } from 'vue-draggable-plus'
import WaypointItem from './BasicWaypointItem.vue'
import CourseMapModal from './CourseMapModal.vue'
import ConfirmModal from './ConfirmModal.vue'
import { useScienceWaypointStore } from '@/stores/scienceWaypoints'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import { scienceWaypointsAPI, recordingAPI } from '@/utils/api'
import type { GpsFixMessage } from '@/types/coordinates'

const scienceStore = useScienceWaypointStore()
const { clickPoint, nextName } = storeToRefs(scienceStore)

const websocketStore = useWebsocketStore()

const rover_latitude_deg = ref(0)
const rover_longitude_deg = ref(0)
const rover_altitude_m = ref<number | null>(null)

const name = computed({
  get: () => nameOverride.value ?? nextName.value,
  set: (val: string) => { nameOverride.value = val },
})
const nameOverride = ref<string | null>(null)
const input = ref({
  lat: { d: null as number | null },
  lon: { d: null as number | null },
  alt: null as number | null,
})

const canAddWaypoint = computed(() =>
  name.value.trim().length > 0 &&
  input.value.lat.d !== null && Number.isFinite(input.value.lat.d) &&
  input.value.lon.d !== null && Number.isFinite(input.value.lon.d) &&
  input.value.alt !== null && Number.isFinite(input.value.alt)
)

const showCourseMapModal = ref(false)
const resetTableModal = ref<InstanceType<typeof ConfirmModal> | null>(null)

const isRecording = ref(false)

async function startRecording() {
  try {
    const recordingName = `Science Recording ${new Date().toLocaleString()}`
    const response = await recordingAPI.create(recordingName, false)
    if (response.status === 'success' && response.recording_id) {
      isRecording.value = true
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
    isRecording.value = false
  }
}

websocketStore.onMessage<GpsFixMessage>('nav', 'gps_fix', (msg) => {
  rover_latitude_deg.value = msg.latitude
  rover_longitude_deg.value = msg.longitude
  rover_altitude_m.value = msg.altitude
})

watch(clickPoint, (pt) => {
  input.value.lat.d = pt.lat
  input.value.lon.d = pt.lon
})

onMounted(() => {
  scienceStore.fetchAll()
})

async function handleDropWaypointAtRover() {
  let altitude = rover_altitude_m.value

  if (altitude === null) {
    try {
      const snap = await scienceWaypointsAPI.getGpsSnapshot()
      if (snap.status === 'success') {
        altitude = snap.altitude ?? null
      }
    } catch {
      altitude = null
    }
  }

  if (altitude === null) return

  try {
    await scienceStore.addWaypoint({
      name: name.value,
      lat: rover_latitude_deg.value,
      lon: rover_longitude_deg.value,
      altitude,
    })
    nameOverride.value = null
  } catch (error) {
    console.error('Failed to add waypoint:', error)
  }
}

async function handleAddWaypoint(coord: { lat: { d: number | null }; lon: { d: number | null }; alt: number | null }) {
  if (!canAddWaypoint.value) return
  try {
    await scienceStore.addWaypoint({
      name: name.value,
      lat: coord.lat.d as number,
      lon: coord.lon.d as number,
      altitude: coord.alt as number,
    })
    nameOverride.value = null
  } catch (error) {
    console.error('Failed to add waypoint:', error)
  }
}

function handleDelete(payload: { index: number }) {
  scienceStore.deleteWaypoint(payload.index)
}

async function handleConfirmResetTable() {
  try {
    await scienceStore.resetTable()
  } catch (error) {
    console.error('Failed to reset waypoints table:', error)
  }
}
</script>

<script lang="ts">
export default {
  name: 'ScienceWaypointEditor',
}
</script>

<style scoped>
.input-group-text {
  justify-content: center;
  min-width: 40px;
  font-size: 0.75rem;
}

.field-label {
  min-width: 48px;
}

.btn-icon {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  width: 1.75rem;
  height: 1.75rem;
  padding: 0;
  font-size: 0.75rem;
}

.data-label {
  font-size: 0.6875rem;
  color: var(--text-muted);
  text-transform: uppercase;
  letter-spacing: 0.02em;
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

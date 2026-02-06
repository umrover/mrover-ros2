<template>
  <div
    v-if="show"
    class="modal-backdrop d-flex align-items-center justify-content-center"
    @click.self="$emit('close')"
  >
    <div class="modal-dialog modal-xl w-90">
      <div class="modal-content bg-theme-card rounded shadow">
        <div class="modal-header border-bottom p-2">
          <h4 class="mb-0">Recordings</h4>
          <button
            type="button"
            class="btn-close me-1"
            @click="$emit('close')"
          ></button>
        </div>
        <div class="modal-body p-0">
          <ul class="nav nav-tabs px-3 pt-3">
            <li class="nav-item">
              <button
                class="nav-link"
                :class="{ active: activeTab === 'rover' }"
                @click="activeTab = 'rover'"
              >
                Rover
              </button>
            </li>
            <li class="nav-item">
              <button
                class="nav-link"
                :class="{ active: activeTab === 'drone' }"
                @click="activeTab = 'drone'"
              >
                Drone
              </button>
            </li>
          </ul>
          <div class="p-3">
            <div class="d-flex gap-3" style="height: 600px">
              <div class="border rounded p-2 overflow-auto sidebar">
                <div
                  v-if="isLoadingRecordings"
                  class="d-flex justify-content-center align-items-center p-4"
                >
                  <div class="spinner-border text-primary" role="status">
                    <span class="visually-hidden">Loading...</span>
                  </div>
                </div>
                <div
                  v-else-if="filteredRecordings.length === 0"
                  class="text-muted"
                >
                  No recordings available
                </div>
                <div
                  v-else
                  v-for="recording in filteredRecordings"
                  :key="recording.id"
                  class="p-2 mb-2 border rounded"
                  :class="
                    selectedRecording?.id === recording.id
                      ? 'bg-primary text-white'
                      : 'hover-bg-light'
                  "
                >
                  <div
                    class="cursor-pointer"
                    @click="selectRecording(recording.id)"
                  >
                    <div class="fw-bold">{{ recording.name }}</div>
                    <small>{{ formatDate(recording.created_at) }}</small>
                    <div>
                      <small>{{ recording.waypoint_count }} waypoints</small>
                    </div>
                  </div>
                  <button
                    class="btn btn-sm btn-danger mt-2 w-100"
                    @click.stop="deleteRecording(recording.id)"
                  >
                    Delete
                  </button>
                </div>
              </div>
              <div class="flex-fill d-flex flex-column gap-2 map-container">
                <div
                  class="border rounded flex-fill position-relative"
                  style="min-height: 400px"
                >
                  <div
                    v-if="isLoadingWaypoints"
                    class="d-flex justify-content-center align-items-center h-100"
                  >
                    <div
                      class="spinner-border text-primary"
                      role="status"
                      style="width: 3rem; height: 3rem"
                    >
                      <span class="visually-hidden">Loading...</span>
                    </div>
                  </div>
                  <div
                    v-else-if="selectedRecording && waypoints.length === 0"
                    class="d-flex align-items-center justify-content-center h-100 p-4"
                  >
                    <div class="alert alert-warning mb-0" role="alert">
                      <h5 class="alert-heading">No GPS Waypoints Recorded</h5>
                      <p class="mb-0">
                        This recording doesn't contain any GPS waypoints.
                      </p>
                      <p class="mb-0">
                        If not testing on rover, make sure sim is running.
                      </p>
                      <p class="mb-0">
                        If testing on rover, ensure GPS can get fix and is publishing.
                      </p>
                    </div>
                  </div>
                  <l-map
                    v-else-if="selectedRecording && waypoints.length > 0"
                    ref="mapRef"
                    class="w-100 h-100"
                    :zoom="19"
                    :center="mapCenter"
                    :options="{ scrollWheelZoom: true }"
                  >
                    <l-tile-layer
                      url="http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}"
                      :options="{
                        maxNativeZoom: 22,
                        maxZoom: 100,
                        subdomains: ['mt0', 'mt1', 'mt2', 'mt3'],
                      }"
                    />
                    <!-- Full path polyline -->
                    <l-polyline
                      :lat-lngs="pathLatLngs"
                      :color="'#2563eb'"
                      :weight="4"
                      :opacity="0.8"
                    />
                    <!-- Start marker (green) -->
                    <l-marker :lat-lng="startLatLng" :icon="startIcon">
                      <l-tooltip>Start</l-tooltip>
                    </l-marker>
                    <!-- End marker (red) -->
                    <l-marker :lat-lng="endLatLng" :icon="endIcon">
                      <l-tooltip>End</l-tooltip>
                    </l-marker>
                    <!-- Current position marker (orange) -->
                    <l-marker :lat-lng="currentLatLng" :icon="currentIcon" />
                  </l-map>
                </div>
                <div
                  v-if="
                    selectedRecording &&
                    waypoints.length > 0 &&
                    !isLoadingWaypoints
                  "
                  class="border rounded p-2"
                >
                  <div class="d-flex align-items-center gap-2 mb-2">
                    <button
                      class="btn btn-sm"
                      :class="isPlaying ? 'btn-danger' : 'btn-success'"
                      @click="togglePlayback"
                    >
                      {{ isPlaying ? 'Pause' : 'Play' }}
                    </button>
                    <button
                      class="btn btn-sm btn-secondary"
                      @click="resetPlayback"
                    >
                      Reset
                    </button>
                    <span class="ms-2"
                      >{{ currentWaypointIndex + 1 }} /
                      {{ waypoints.length }}</span
                    >
                  </div>
                  <input
                    type="range"
                    class="form-range mb-2"
                    :min="0"
                    :max="waypoints.length - 1"
                    v-model.number="currentWaypointIndex"
                  />
                  <div
                    class="d-flex justify-content-between text-muted"
                    style="font-size: 12px"
                  >
                    <div>
                      <strong>Start:</strong>
                      {{ formatTimestamp(startTimestamp) }}
                    </div>
                    <div>
                      <strong>Current:</strong>
                      {{ formatTimestamp(currentTimestamp) }}
                    </div>
                    <div>
                      <strong>End:</strong> {{ formatTimestamp(endTimestamp) }}
                    </div>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, shallowRef, computed, watch, onBeforeUnmount, nextTick } from 'vue'
import {
  LMap,
  LTileLayer,
  LMarker,
  LPolyline,
  LTooltip,
} from '@vue-leaflet/vue-leaflet'
import 'leaflet/dist/leaflet.css'
import L from 'leaflet'
import { recordingAPI } from '@/utils/api'
import type { Recording, RecordedWaypoint } from '@/utils/apiTypes'

const props = defineProps({
  show: {
    type: Boolean,
    required: true,
  },
})

defineEmits(['close'])

const activeTab = ref<'rover' | 'drone'>('rover')
const recordings = ref<Recording[]>([])
const selectedRecording = ref<Recording | null>(null)
const waypoints = shallowRef<RecordedWaypoint[]>([])
const currentWaypointIndex = ref(0)
const isPlaying = ref(false)
const mapRef = ref<{ leafletObject: L.Map } | null>(null)
const isLoadingRecordings = ref(false)
const isLoadingWaypoints = ref(false)
let playbackInterval: number | null = null
const startIcon = L.divIcon({
  html: '<div style="background-color: #10b981; width: 12px; height: 12px; border-radius: 50%; border: 2px solid white;"></div>',
  className: 'custom-div-icon',
  iconSize: [12, 12],
  iconAnchor: [6, 6],
  iconUrl: '',
}) as L.Icon

const endIcon = L.divIcon({
  html: '<div style="background-color: #ef4444; width: 12px; height: 12px; border-radius: 50%; border: 2px solid white;"></div>',
  className: 'custom-div-icon',
  iconSize: [12, 12],
  iconAnchor: [6, 6],
  iconUrl: '',
}) as L.Icon

const currentIcon = L.divIcon({
  html: '<div style="background-color: #f59e0b; width: 16px; height: 16px; border-radius: 50%; border: 3px solid white; box-shadow: 0 2px 4px rgba(0,0,0,0.3);"></div>',
  className: 'custom-div-icon',
  iconSize: [16, 16],
  iconAnchor: [8, 8],
  iconUrl: '',
}) as L.Icon

const filteredRecordings = computed(() => {
  return recordings.value.filter(r =>
    activeTab.value === 'drone' ? r.is_drone : !r.is_drone,
  )
})

const mapCenter = computed<[number, number]>(() => {
  const first = waypoints.value[0]
  if (!first) return [0, 0]
  return [first.lat, first.lon]
})

const pathLatLngs = computed<[number, number][]>(() => {
  return waypoints.value.map(wp => [wp.lat, wp.lon])
})

const startLatLng = computed<[number, number]>(() => {
  const first = waypoints.value[0]
  if (!first) return [0, 0]
  return [first.lat, first.lon]
})

const endLatLng = computed<[number, number]>(() => {
  const last = waypoints.value[waypoints.value.length - 1]
  if (!last) return [0, 0]
  return [last.lat, last.lon]
})

const currentLatLng = computed<[number, number]>(() => {
  const wp = waypoints.value[currentWaypointIndex.value]
  if (!wp) return [0, 0]
  return [wp.lat, wp.lon]
})

const currentTimestamp = computed(() => {
  const wp = waypoints.value[currentWaypointIndex.value]
  return wp?.timestamp ?? ''
})

const startTimestamp = computed(() => {
  const first = waypoints.value[0]
  return first?.timestamp ?? ''
})

const endTimestamp = computed(() => {
  const last = waypoints.value[waypoints.value.length - 1]
  return last?.timestamp ?? ''
})

const formatTimestamp = (isoString: string) => {
  if (!isoString) return ''
  return new Date(isoString).toISOString()
}

const loadRecordings = async () => {
  isLoadingRecordings.value = true
  try {
    const data = await recordingAPI.getAll()
    if (data.status === 'success' && data.recordings) {
      recordings.value = data.recordings
    }
  } catch (error) {
    console.error('Failed to load recordings:', error)
  } finally {
    isLoadingRecordings.value = false
  }
}

const selectRecording = async (recordingId: number) => {
  isLoadingWaypoints.value = true
  try {
    const recording = recordings.value.find(r => r.id === recordingId)
    if (!recording) return

    selectedRecording.value = recording
    const data = await recordingAPI.getWaypoints(recordingId)
    if (data.status === 'success' && data.waypoints) {
      waypoints.value = data.waypoints
      currentWaypointIndex.value = 0
      isPlaying.value = false
      if (playbackInterval) clearInterval(playbackInterval)
      playbackInterval = null

      await nextTick()
      if (mapRef.value && waypoints.value.length > 0) {
        const map = mapRef.value.leafletObject
        const bounds = L.latLngBounds(
          waypoints.value.map(wp => [wp.lat, wp.lon]),
        )
        map.fitBounds(bounds, { padding: [30, 30], maxZoom: 19 })
      }
    }
  } catch (error) {
    console.error('Failed to load waypoints:', error)
  } finally {
    isLoadingWaypoints.value = false
  }
}

const togglePlayback = () => {
  isPlaying.value = !isPlaying.value

  if (isPlaying.value) {
    if (currentWaypointIndex.value >= waypoints.value.length - 1) {
      currentWaypointIndex.value = 0
    }

    playbackInterval = window.setInterval(() => {
      if (currentWaypointIndex.value < waypoints.value.length - 1) {
        currentWaypointIndex.value++
      } else {
        isPlaying.value = false
        if (playbackInterval) clearInterval(playbackInterval)
        playbackInterval = null
      }
    }, 100)
  } else {
    if (playbackInterval) clearInterval(playbackInterval)
    playbackInterval = null
  }
}

const resetPlayback = () => {
  currentWaypointIndex.value = 0
  isPlaying.value = false
  if (playbackInterval) clearInterval(playbackInterval)
  playbackInterval = null
}

const formatDate = (dateString: string) => {
  return new Date(dateString).toLocaleString()
}

const deleteRecording = async (recordingId: number) => {
  if (
    !confirm(
      'Are you sure you want to delete this recording? This cannot be undone.',
    )
  ) {
    return
  }

  try {
    await recordingAPI.delete(recordingId)

    if (selectedRecording.value?.id === recordingId) {
      selectedRecording.value = null
      waypoints.value = []
      currentWaypointIndex.value = 0
      isPlaying.value = false
      if (playbackInterval) clearInterval(playbackInterval)
      playbackInterval = null
    }

    await loadRecordings()
  } catch (error) {
    console.error('Failed to delete recording:', error)
    alert('Failed to delete recording. Please try again.')
  }
}

watch(
  () => props.show,
  async newVal => {
    if (newVal) {
      selectedRecording.value = null
      waypoints.value = []
      currentWaypointIndex.value = 0
      isPlaying.value = false
      if (playbackInterval) clearInterval(playbackInterval)
      playbackInterval = null

      await loadRecordings()
    } else {
      if (playbackInterval) clearInterval(playbackInterval)
      playbackInterval = null
      isPlaying.value = false

      selectedRecording.value = null
      waypoints.value = []
      currentWaypointIndex.value = 0
    }
  },
)

watch(activeTab, () => {
  selectedRecording.value = null
  waypoints.value = []
  currentWaypointIndex.value = 0
  isPlaying.value = false
  if (playbackInterval) clearInterval(playbackInterval)
  playbackInterval = null
})

onBeforeUnmount(() => {
  if (playbackInterval) clearInterval(playbackInterval)
})
</script>

<style scoped>
.modal-backdrop {
  position: fixed;
  inset: 0;
  background-color: rgba(0, 0, 0, 0.5);
  z-index: 1050;
}

.w-90 {
  width: 90%;
  max-width: 1400px;
}

.sidebar {
  width: 280px;
  min-width: 280px;
  max-width: 280px;
}

.map-container {
  min-width: 0;
  flex: 1;
}

.cursor-pointer {
  cursor: pointer;
}

.hover-bg-light:hover {
  background-color: var(--view-bg);
}
</style>

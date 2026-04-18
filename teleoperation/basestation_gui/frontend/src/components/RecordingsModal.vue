<template>
  <Teleport to="body">
    <div
      v-if="show"
      class="modal-backdrop"
      @click.self="$emit('close')"
    >
      <div class="modal-dialog recordings-dialog">
        <div class="modal-content h-full flex flex-col overflow-hidden">
          <div class="modal-header">
            <h5 class="modal-title">Recordings Library</h5>
            <div class="flex items-center gap-2">
              <button
                class="btn btn-sm btn-danger"
                @click="clearAllRecordingsModal?.open()"
              >
                Clear All
              </button>
              <button
                type="button"
                class="btn-close"
                @click="$emit('close')"
              >
                <i class="bi bi-x-lg"></i>
              </button>
            </div>
          </div>

          <div class="modal-body p-3 flex flex-col flex-1 min-h-0">
            <!-- Tab Navigation -->
            <div class="flex gap-2 mb-4">
              <button
                class="nav-tab-btn"
                :class="activeTab === 'rover' ? 'btn-primary' : 'btn-outline-secondary'"
                @click="activeTab = 'rover'"
              >
                Rover Sessions
              </button>
              <button
                class="nav-tab-btn"
                :class="activeTab === 'drone' ? 'btn-primary' : 'btn-outline-secondary'"
                @click="activeTab = 'drone'"
              >
                Drone Sessions
              </button>
            </div>

            <div class="flex flex-1 gap-4 min-h-0">
              <!-- Sidebar Column -->
              <div class="flex flex-col w-80 min-w-0">
                <div class="p-2 flex justify-between items-center">
                  <h4 class="component-header">Sessions</h4>
                </div>
                <div class="waypoint-wrapper p-2 rounded grow overflow-auto relative scroll border">
                  <div
                    v-if="isLoadingRecordings"
                    class="flex justify-center items-center p-6"
                  >
                    <div class="spinner text-primary" role="status"></div>
                  </div>
                  <div
                    v-else-if="filteredRecordings.length === 0"
                    class="course-empty-state"
                  >
                    <span>No recordings available</span>
                  </div>
                  <div
                    v-else
                    v-for="recording in filteredRecordings"
                    :key="recording.id"
                    class="list-item cursor-pointer"
                    :class="{ 'recording-selected': selectedRecording?.id === recording.id }"
                    @click="selectRecording(recording.id)"
                  >
                    <div class="flex justify-between items-start">
                      <div class="flex flex-col">
                        <span class="list-item-title">{{ recording.name }}</span>
                        <span class="data-label">{{ formatDate(recording.created_at) }}</span>
                      </div>
                      <button
                        class="btn btn-sm btn-danger btn-icon flex-shrink-0 !bg-theme-danger"
                        @click.stop="deleteRecording(recording.id)"
                      >
                        <i class="bi bi-trash-fill"></i>
                      </button>
                    </div>
                    <div class="mt-1">
                      <span class="data-label font-bold text-primary">{{ recording.waypoint_count }} Waypoints</span>
                    </div>
                  </div>
                </div>
              </div>

              <!-- Content Column -->
              <div class="flex flex-col flex-1 min-w-0">
                <div class="p-2 flex justify-between items-center">
                  <h4 class="component-header">Path Visualization</h4>
                  <div class="flex gap-2">
                    <button
                      class="btn btn-sm"
                      :class="showCourseWaypoints ? 'btn-success' : 'btn-danger'"
                      @click="showCourseWaypoints = !showCourseWaypoints"
                    >
                      <i class="bi" :class="showCourseWaypoints ? 'bi-eye-fill' : 'bi-eye-slash'"></i> 
                      Course
                    </button>
                    <div class="h-8 w-px bg-panel-border mx-1"></div>
                    <button
                      class="btn btn-sm btn-info"
                      :disabled="erdStore.waypoints.length === 0"
                      @click="erdStore.exportToText()"
                    >
                      <i class="bi bi-download"></i> TXT
                    </button>
                    <button
                      class="btn btn-sm btn-primary"
                      :disabled="!selectedRecording || waypoints.length === 0 || isDownloading"
                      @click="downloadMapPNG"
                    >
                      <i class="bi bi-download"></i> PNG
                    </button>
                  </div>
                </div>
                <div class="flex-1 flex flex-col gap-2 min-h-0">
                  <div ref="mapCaptureRef" class="waypoint-wrapper p-0 rounded grow relative overflow-hidden border">
                    <div
                      v-if="isLoadingWaypoints"
                      class="flex justify-center items-center h-full bg-theme-view/50 z-10"
                    >
                      <div class="spinner text-primary"></div>
                    </div>
                    <div
                      v-else-if="selectedRecording && waypoints.length === 0"
                      class="flex items-center justify-center h-full p-6"
                    >
                      <div class="alert alert-warning mb-0 shadow-sm" role="alert">
                        <h5 class="alert-heading font-bold">No GPS Waypoints</h5>
                        <p class="mb-0 text-sm">This recording doesn't contain any GPS data.</p>
                      </div>
                    </div>
                    <div v-else-if="!selectedRecording" class="course-empty-state">
                      <span class="uppercase tracking-widest opacity-50 font-bold">Select a session to view</span>
                    </div>
                    <l-map
                      v-else-if="selectedRecording && waypoints.length > 0"
                      ref="mapRef"
                      class="w-full h-full"
                      :zoom="19"
                      :center="mapCenter"
                      :options="{ scrollWheelZoom: true }"
                    >
                      <l-tile-layer :url="onlineUrl" :options="onlineTileOptions" />
                      
                      <!-- Course Waypoints Overlay -->
                      <template v-if="showCourseWaypoints">
                        <l-marker
                          v-for="wp in erdStore.waypoints"
                          :key="wp.db_id"
                          :lat-lng="[wp.lat, wp.lon]"
                          :icon="waypointIconSmall"
                        >
                          <l-tooltip>{{ wp.name }} (Course)</l-tooltip>
                        </l-marker>
                      </template>

                      <l-polyline :lat-lngs="pathLatLngs" :color="'var(--accent)'" :weight="4" :opacity="0.8" />
                      
                      <!-- Start/End markers -->
                      <l-marker :lat-lng="startLatLng" :icon="startIcon"><l-tooltip>Start</l-tooltip></l-marker>
                      <l-marker :lat-lng="endLatLng" :icon="endIcon"><l-tooltip>End</l-tooltip></l-marker>
                      
                      <!-- Playback Current Location Marker (Dot for both) -->
                      <l-marker 
                        :lat-lng="currentLatLng" 
                        :icon="activeTab === 'drone' ? droneDotIcon : roverDotIcon"
                        :rotation-angle="currentBearing"
                      />
                    </l-map>
                  </div>

                  <!-- Playback Panel -->
                  <div
                    v-if="selectedRecording && waypoints.length > 0 && !isLoadingWaypoints"
                    class="list-item !m-0 shadow-sm"
                  >
                    <div class="flex items-center gap-3 mb-3">
                      <button
                        class="btn btn-sm w-24 btn-success"
                        @click="togglePlayback"
                      >
                        {{ isPlaying ? 'Pause' : 'Play' }}
                      </button>
                      <button
                        class="btn btn-sm btn-outline-secondary"
                        @click="resetPlayback"
                      >
                        Reset
                      </button>
                      <div class="h-4 w-px bg-panel-border mx-1"></div>
                      <span class="list-item-title">
                        Step {{ currentWaypointIndex + 1 }} / {{ waypoints.length }}
                      </span>
                    </div>
                    <input
                      type="range"
                      class="form-range mb-3"
                      :min="0"
                      :max="waypoints.length - 1"
                      v-model.number="currentWaypointIndex"
                    />
                    <div class="grid grid-cols-3 gap-4 text-left">
                      <div class="flex flex-col">
                        <span class="data-label text-[0.6rem]">Session Start</span>
                        <span class="text-xs font-mono bg-theme-view p-1 rounded text-center">{{ formatTimestamp(startTimestamp) }}</span>
                      </div>
                      <div class="flex flex-col">
                        <span class="data-label text-primary text-[0.6rem]">Current Time</span>
                        <span class="text-xs font-mono font-bold bg-primary-subtle p-1 rounded border border-primary/20 text-center text-primary">{{ formatTimestamp(currentTimestamp) }}</span>
                      </div>
                      <div class="flex flex-col">
                        <span class="data-label text-[0.6rem]">Session End</span>
                        <span class="text-xs font-mono bg-theme-view p-1 rounded text-center">{{ formatTimestamp(endTimestamp) }}</span>
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
  </Teleport>

  <ConfirmModal
    ref="clearAllRecordingsModal"
    modal-id="clearAllRecordingsModal"
    title="Clear All Recordings"
    message="Are you sure you want to delete all recordings? This cannot be undone."
    confirm-text="Clear All"
    @confirm="clearAll"
  />
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
import 'leaflet-rotatedmarker'
import { recordingAPI } from '@/utils/api'
import type { Recording, RecordedWaypoint } from '@/utils/apiTypes'
import { useRoverMap } from '@/composables/useRoverMap'
import { useErdStore } from '@/stores/erd'
import ConfirmModal from './ConfirmModal.vue'
import { toPng } from 'html-to-image'

const props = defineProps({
  show: {
    type: Boolean,
    required: true,
  },
})

defineEmits(['close'])

const {
  mapRef,
  onlineUrl,
  onlineTileOptions,
  getMap,
} = useRoverMap()

const erdStore = useErdStore()

const activeTab = ref<'rover' | 'drone'>('rover')
const recordings = ref<Recording[]>([])
const selectedRecording = ref<Recording | null>(null)
const waypoints = shallowRef<RecordedWaypoint[]>([])
const currentWaypointIndex = ref(0)
const isPlaying = ref(false)
const isLoadingRecordings = ref(false)
const isLoadingWaypoints = ref(false)
const isDownloading = ref(false)
const showCourseWaypoints = ref(false)
const mapCaptureRef = ref<HTMLElement | null>(null)
let playbackInterval: number | null = null

const clearAllRecordingsModal = ref<InstanceType<typeof ConfirmModal> | null>(null)

const startIcon = L.divIcon({
  html: '<div class="map-marker-dot map-marker-start"></div>',
  className: 'custom-div-icon',
  iconSize: [12, 12],
  iconAnchor: [6, 6],
  iconUrl: '',
}) as L.Icon

const endIcon = L.divIcon({
  html: '<div class="map-marker-dot map-marker-end"></div>',
  className: 'custom-div-icon',
  iconSize: [12, 12],
  iconAnchor: [6, 6],
  iconUrl: '',
}) as L.Icon

const roverDotIcon = L.divIcon({
  html: '<div class="map-marker-dot map-marker-rover-dot"></div>',
  className: 'custom-div-icon',
  iconSize: [16, 16],
  iconAnchor: [8, 8],
  iconUrl: '',
}) as L.Icon

const droneDotIcon = L.divIcon({
  html: '<div class="map-marker-dot map-marker-drone-dot"></div>',
  className: 'custom-div-icon',
  iconSize: [16, 16],
  iconAnchor: [8, 8],
  iconUrl: '',
}) as L.Icon

const waypointIconSmall = L.icon({
  iconUrl: '/waypoint_marker.svg',
  iconSize: [40, 40],
  iconAnchor: [20, 40],
  popupAnchor: [0, -20],
})

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

const currentBearing = computed(() => {
  const wp = waypoints.value[currentWaypointIndex.value]
  return wp?.bearing ?? 0
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
  if (!isoString) return '--:--:--'
  const date = new Date(isoString)
  return date.toLocaleTimeString([], { hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit' })
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
  if (selectedRecording.value?.id === recordingId) return
  
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
      const map = getMap()
      if (map && waypoints.value.length > 0) {
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

const clearAll = async () => {
  try {
    await recordingAPI.deleteAll()
    selectedRecording.value = null
    waypoints.value = []
    currentWaypointIndex.value = 0
    isPlaying.value = false
    if (playbackInterval) clearInterval(playbackInterval)
    playbackInterval = null
    await loadRecordings()
  } catch (error) {
    console.error('Failed to clear recordings:', error)
  }
}

const sanitizeFilename = (title: string): string => {
  return title.replace(/[^a-zA-Z0-9]+/g, '_').replace(/^_|_$/g, '')
}

const downloadMapPNG = async () => {
  if (!mapCaptureRef.value || !selectedRecording.value) return
  
  isDownloading.value = true
  try {
    // Small delay to ensure everything is rendered
    await nextTick()
    const dataUrl = await toPng(mapCaptureRef.value, {
      cacheBust: true,
      backgroundColor: '#dddddd', // Matches var(--view-bg) in light theme
    })
    const link = document.createElement('a')
    link.download = `${sanitizeFilename(selectedRecording.value.name)}.png`
    link.href = dataUrl
    link.click()
  } catch (err) {
    console.error('Failed to capture map image:', err)
  } finally {
    isDownloading.value = false
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
.recordings-dialog {
  max-width: 1200px;
  width: 90%;
  height: 80vh;
}

.waypoint-wrapper {
  background-color: var(--view-bg);
}

.nav-tab-btn {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  height: var(--btn-height-md);
  padding: 0 1rem;
  font-size: 0.75rem;
  font-weight: 700;
  text-transform: uppercase;
  letter-spacing: 0.05em;
  cursor: pointer;
  border-radius: var(--radius-sm);
  transition: all var(--transition);
}

.recording-selected {
  outline: 2px solid var(--accent);
  outline-offset: -2px;
  background-color: var(--card-bg) !important;
  box-shadow: var(--shadow-sm);
  transition: none !important;
}

.list-item-title {
  font-size: 0.875rem;
  font-weight: 600;
}

.data-label {
  font-size: 0.6875rem;
  color: var(--text-muted);
  text-transform: uppercase;
  letter-spacing: 0.02em;
}

.bg-primary-subtle {
  background-color: rgba(var(--accent-rgb), 0.1);
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

.scroll {
  scrollbar-gutter: stable;
}
</style>

<style>
.map-marker-dot {
  border-radius: 50%;
  border: 2px solid var(--card-bg);
}

.map-marker-start {
  width: 12px;
  height: 12px;
  background-color: var(--status-ok);
}

.map-marker-end {
  width: 12px;
  height: 12px;
  background-color: var(--status-error);
}

.map-marker-rover-dot {
  width: 16px;
  height: 16px;
  background-color: var(--accent);
  border: 2px solid #fff;
  box-shadow: var(--shadow-sm);
}

.map-marker-drone-dot {
  width: 16px;
  height: 16px;
  background-color: var(--status-info);
  border: 2px solid #fff;
  box-shadow: var(--shadow-sm);
}

.map-marker-current {
  width: 16px;
  height: 16px;
  border-width: 3px;
  background-color: var(--status-warn);
  box-shadow: var(--shadow-md);
}
</style>

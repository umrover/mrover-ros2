<template>
  <Teleport to="body">
    <div
      v-if="show"
      class="modal-backdrop flex items-center justify-center"
      @click.self="$emit('close')"
    >
      <div class="modal-dialog modal-xl" data-testid="pw-recordings-modal" style="width: 90%; max-width: 1400px">
        <div class="modal-content bg-theme-card rounded shadow">
        <div class="flex justify-between items-center px-4 py-2 border-b border-2">
          <h4 class="component-header">Recordings</h4>
          <button
            type="button"
            class="cmd-btn cmd-btn-sm cmd-btn-outline-control"
            @click="$emit('close')"
          >
            <i class="bi bi-x-lg"></i>
          </button>
        </div>
        <div class="modal-body p-0">
          <ul class="cmd-nav-tabs px-4 pt-2 gap-1 border-0">
            <li class="nav-item">
              <button
                class="nav-link-tab"
                :class="{ active: activeTab === 'rover' }"
                data-testid="pw-recordings-tab-rover"
                @click="activeTab = 'rover'"
              >
                Rover
              </button>
            </li>
            <li class="nav-item">
              <button
                class="nav-link-tab"
                :class="{ active: activeTab === 'drone' }"
                data-testid="pw-recordings-tab-drone"
                @click="activeTab = 'drone'"
              >
                Drone
              </button>
            </li>
          </ul>
          <div class="p-4">
            <div class="flex gap-4" style="height: 600px">
              <div class="recordings-sidebar p-2 rounded overflow-auto">
                <div
                  v-if="isLoadingRecordings"
                  class="flex justify-center items-center p-6"
                >
                  <div class="cmd-spinner text-cmd-primary" role="status">
                    <span class="sr-only">Loading...</span>
                  </div>
                </div>
                <div
                  v-else-if="filteredRecordings.length === 0"
                  class="text-muted text-center p-4 text-sm"
                >
                  No recordings available
                </div>
                <div
                  v-else
                  v-for="recording in filteredRecordings"
                  :key="recording.id"
                  class="cmd-list-item"
                  :class="{ 'cmd-list-item--selected': selectedRecording?.id === recording.id }"
                >
                  <div
                    role="button"
                    @click="selectRecording(recording.id)"
                  >
                    <div class="font-semibold text-sm">{{ recording.name }}</div>
                    <div class="text-muted" style="font-size: 0.6875rem">{{ formatDate(recording.created_at) }}</div>
                    <div class="text-muted" style="font-size: 0.6875rem">{{ recording.waypoint_count }} waypoints</div>
                  </div>
                  <button
                    class="cmd-btn cmd-btn-sm cmd-btn-outline-secondary w-full mt-2"
                    @click.stop="deleteRecording(recording.id)"
                  >
                    Delete
                  </button>
                </div>
              </div>
              <div class="flex-1 flex flex-col gap-2" style="min-width: 0">
                <div
                  class="border border-2 rounded flex-1 relative"
                  style="min-height: 400px"
                >
                  <div
                    v-if="isLoadingWaypoints"
                    class="flex justify-center items-center h-full"
                  >
                    <div
                      class="cmd-spinner text-cmd-primary"
                      role="status"
                      style="width: 3rem; height: 3rem"
                    >
                      <span class="sr-only">Loading...</span>
                    </div>
                  </div>
                  <div
                    v-else-if="selectedRecording && waypoints.length === 0"
                    class="flex items-center justify-center h-full p-6"
                  >
                    <div class="cmd-alert cmd-alert-warning mb-0" role="alert">
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
                    class="w-full h-full"
                    :zoom="19"
                    :center="mapCenter"
                    :options="{ scrollWheelZoom: true }"
                  >
                    <l-tile-layer
                      :url="onlineUrl"
                      :options="onlineTileOptions"
                    />
                    <l-polyline
                      :lat-lngs="pathLatLngs"
                      :color="'var(--cmd-accent)'"
                      :weight="4"
                      :opacity="0.8"
                    />
                    <l-marker :lat-lng="startLatLng" :icon="startIcon">
                      <l-tooltip>Start</l-tooltip>
                    </l-marker>
                    <l-marker :lat-lng="endLatLng" :icon="endIcon">
                      <l-tooltip>End</l-tooltip>
                    </l-marker>
                    <l-marker :lat-lng="currentLatLng" :icon="currentIcon" />
                  </l-map>
                </div>
                <div
                  v-if="selectedRecording && waypoints.length > 0 && !isLoadingWaypoints"
                  class="p-2 rounded"
                  style="background-color: var(--view-bg)"
                >
                  <div class="flex items-center gap-2 mb-2">
                    <button
                      class="cmd-btn cmd-btn-sm"
                      :class="isPlaying ? 'cmd-btn-outline-secondary' : 'cmd-btn-outline-control'"
                      @click="togglePlayback"
                    >
                      {{ isPlaying ? 'Pause' : 'Play' }}
                    </button>
                    <button
                      class="cmd-btn cmd-btn-sm cmd-btn-outline-secondary"
                      @click="resetPlayback"
                    >
                      Reset
                    </button>
                    <span class="font-semibold text-sm ml-2">{{ currentWaypointIndex + 1 }} / {{ waypoints.length }}</span>
                  </div>
                  <input
                    type="range"
                    class="cmd-form-range mb-2"
                    :min="0"
                    :max="waypoints.length - 1"
                    v-model.number="currentWaypointIndex"
                  />
                  <div class="flex justify-between" style="font-size: 0.6875rem">
                    <div>
                      <span class="text-muted mr-1">Start:</span>
                      <span>{{ formatTimestamp(startTimestamp) }}</span>
                    </div>
                    <div>
                      <span class="text-muted mr-1">Current:</span>
                      <span>{{ formatTimestamp(currentTimestamp) }}</span>
                    </div>
                    <div>
                      <span class="text-muted mr-1">End:</span>
                      <span>{{ formatTimestamp(endTimestamp) }}</span>
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
import { useRoverMap } from '@/composables/useRoverMap'

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

const activeTab = ref<'rover' | 'drone'>('rover')
const recordings = ref<Recording[]>([])
const selectedRecording = ref<Recording | null>(null)
const waypoints = shallowRef<RecordedWaypoint[]>([])
const currentWaypointIndex = ref(0)
const isPlaying = ref(false)
const isLoadingRecordings = ref(false)
const isLoadingWaypoints = ref(false)
let playbackInterval: number | null = null
const startIcon = L.divIcon({
  html: '<div style="background-color: var(--cmd-status-ok); width: 12px; height: 12px; border-radius: 50%; border: 2px solid white;"></div>',
  className: 'custom-div-icon',
  iconSize: [12, 12],
  iconAnchor: [6, 6],
  iconUrl: '',
}) as L.Icon

const endIcon = L.divIcon({
  html: '<div style="background-color: var(--cmd-status-error); width: 12px; height: 12px; border-radius: 50%; border: 2px solid white;"></div>',
  className: 'custom-div-icon',
  iconSize: [12, 12],
  iconAnchor: [6, 6],
  iconUrl: '',
}) as L.Icon

const currentIcon = L.divIcon({
  html: '<div style="background-color: var(--cmd-status-warn); width: 16px; height: 16px; border-radius: 50%; border: 3px solid white; box-shadow: 0 2px 4px rgba(0,0,0,0.3);"></div>',
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
  z-index: 1050;
  background-color: var(--cmd-backdrop);
}

.nav-link-tab {
  padding: 0.5rem 1rem;
  font-size: 0.75rem;
  font-weight: 600;
  color: var(--text-muted);
  text-transform: uppercase;
  letter-spacing: 0.03em;
  cursor: pointer;
  background: transparent;
  border: 2px solid transparent;
  border-bottom: none;
  border-radius: var(--cmd-radius-sm) var(--cmd-radius-sm) 0 0;
  transition: all var(--cmd-transition);
}

.nav-link-tab:hover,
.nav-link-tab.active {
  color: var(--cmd-accent);
  background-color: var(--view-bg);
  border-color: var(--cmd-panel-border);
}

.recordings-sidebar {
  flex-shrink: 0;
  width: 280px;
  background-color: var(--view-bg);
}

.cmd-list-item--selected {
  background-color: var(--view-bg);
  border-color: var(--cmd-accent);
}
</style>

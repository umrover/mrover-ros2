<template>
  <div v-if="show" class="modal-backdrop d-flex align-items-center justify-content-center" @click.self="$emit('close')">
    <div class="modal-dialog modal-xl w-90">
      <div class="modal-content bg-white rounded shadow">
        <div class="modal-header border-bottom p-2">
          <h4 class="mb-0">Recordings (IMPL INCOMPLETE)</h4>
          <button type="button" class="btn-close" @click="$emit('close')"></button>
        </div>
        <div class="modal-body p-0">
          <ul class="nav nav-tabs px-3 pt-3">
            <li class="nav-item">
              <button class="nav-link" :class="{ active: activeTab === 'rover' }" @click="activeTab = 'rover'">
                Rover
              </button>
            </li>
            <li class="nav-item">
              <button class="nav-link" :class="{ active: activeTab === 'drone' }" @click="activeTab = 'drone'">
                Drone
              </button>
            </li>
          </ul>
          <div class="p-3">
            <div class="d-flex gap-3" style="height: 600px">
              <div class="border rounded p-2 overflow-auto" style="width: 300px">
                <div v-if="filteredRecordings.length === 0" class="text-muted">
                  No recordings available
                </div>
                <div
                  v-for="recording in filteredRecordings"
                  :key="recording.id"
                  class="p-2 mb-2 border rounded cursor-pointer"
                  :class="selectedRecording?.id === recording.id ? 'bg-primary text-white' : 'hover-bg-light'"
                  @click="selectRecording(recording.id)"
                >
                  <div class="fw-bold">{{ recording.name }}</div>
                  <small>{{ formatDate(recording.created_at) }}</small>
                  <div><small>{{ recording.waypoint_count }} waypoints</small></div>
                </div>
              </div>
              <div class="flex-fill d-flex flex-column gap-2">
                <div ref="mapContainer" class="border rounded flex-fill" style="min-height: 0"></div>
                <div v-if="selectedRecording && waypoints.length > 0" class="border rounded p-2">
                  <div class="d-flex align-items-center gap-2 mb-2">
                    <button class="btn btn-sm" :class="isPlaying ? 'btn-danger' : 'btn-success'" @click="togglePlayback">
                      {{ isPlaying ? 'Pause' : 'Play' }}
                    </button>
                    <button class="btn btn-sm btn-secondary" @click="resetPlayback">Reset</button>
                    <span class="ms-2">{{ currentWaypointIndex + 1 }} / {{ waypoints.length }}</span>
                  </div>
                  <input
                    type="range"
                    class="form-range"
                    :min="0"
                    :max="waypoints.length - 1"
                    v-model.number="currentWaypointIndex"
                    @input="updateMapPosition"
                  />
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
import { ref, computed, watch, onBeforeUnmount, nextTick } from 'vue'
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
const waypoints = ref<RecordedWaypoint[]>([])
const currentWaypointIndex = ref(0)
const isPlaying = ref(false)
const mapContainer = ref<HTMLElement | null>(null)
let map: L.Map | null = null
let marker: L.Marker | null = null
let polyline: L.Polyline | null = null
let playbackInterval: number | null = null

const filteredRecordings = computed(() => {
  return recordings.value.filter(r =>
    activeTab.value === 'drone' ? r.is_drone : !r.is_drone
  )
})

const loadRecordings = async () => {
  try {
    const data = await recordingAPI.getAll()
    if (data.status === 'success' && data.recordings) {
      recordings.value = data.recordings
    }
  } catch (error) {
    console.error('Failed to load recordings:', error)
  }
}

const selectRecording = async (recordingId: number) => {
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
      initializeMap()
    }
  } catch (error) {
    console.error('Failed to load waypoints:', error)
  }
}

const initializeMap = async () => {
  await nextTick()
  if (!mapContainer.value || waypoints.value.length === 0) return

  if (map) map.remove()

  map = L.map(mapContainer.value).setView([waypoints.value[0].lat, waypoints.value[0].lon], 18)

  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '© OpenStreetMap contributors',
  }).addTo(map)

  const latLngs = waypoints.value.map(wp => L.latLng(wp.lat, wp.lon))
  polyline = L.polyline(latLngs, { color: 'blue', weight: 3 }).addTo(map)
  marker = L.marker([waypoints.value[0].lat, waypoints.value[0].lon]).addTo(map)

  map.fitBounds(polyline.getBounds())
}

const updateMapPosition = () => {
  if (!marker || !map || waypoints.value.length === 0) return
  const waypoint = waypoints.value[currentWaypointIndex.value]
  marker.setLatLng(L.latLng(waypoint.lat, waypoint.lon))
  map.panTo(L.latLng(waypoint.lat, waypoint.lon))
}

const togglePlayback = () => {
  isPlaying.value = !isPlaying.value

  if (isPlaying.value) {
    playbackInterval = window.setInterval(() => {
      if (currentWaypointIndex.value < waypoints.value.length - 1) {
        currentWaypointIndex.value++
        updateMapPosition()
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
  updateMapPosition()
}

const formatDate = (dateString: string) => {
  return new Date(dateString).toLocaleString()
}

watch(() => props.show, async (newVal) => {
  if (newVal) {
    await loadRecordings()
  } else {
    if (playbackInterval) clearInterval(playbackInterval)
    playbackInterval = null
    isPlaying.value = false
  }
})

watch(activeTab, () => {
  selectedRecording.value = null
  waypoints.value = []
  currentWaypointIndex.value = 0
  isPlaying.value = false
  if (playbackInterval) clearInterval(playbackInterval)
  playbackInterval = null
  if (map) {
    map.remove()
    map = null
  }
})

onBeforeUnmount(() => {
  if (playbackInterval) clearInterval(playbackInterval)
  if (map) map.remove()
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
  max-width: 1200px;
}

.cursor-pointer {
  cursor: pointer;
}

.hover-bg-light:hover {
  background-color: #f8f9fa;
}
</style>

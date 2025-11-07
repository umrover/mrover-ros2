<template>
  <Transition name="modal">
    <div
      v-if="show"
      class="modal-backdrop"
      @click.self="$emit('close')"
    >
      <div class="modal-dialog modal-xl">
        <div class="modal-content">
          <div class="modal-header">
            <h4>Recordings</h4>
            <button
              type="button"
              class="btn-close"
              @click="$emit('close')"
            ></button>
          </div>
          <div class="modal-body p-0">
            <ul class="nav nav-tabs px-3 pt-3" role="tablist">
              <li class="nav-item" role="presentation">
                <button
                  class="nav-link"
                  :class="{ active: activeTab === 'rover' }"
                  @click="activeTab = 'rover'"
                >
                  Rover
                </button>
              </li>
              <li class="nav-item" role="presentation">
                <button
                  class="nav-link"
                  :class="{ active: activeTab === 'drone' }"
                  @click="activeTab = 'drone'"
                >
                  Drone
                </button>
              </li>
            </ul>
            <div class="tab-content p-3">
              <div class="d-flex gap-3" style="height: 600px;">
                <div class="recordings-list border rounded p-2" style="width: 300px; overflow-y: auto;">
                  <div v-if="filteredRecordings.length === 0" class="text-muted">
                    No recordings available
                  </div>
                  <div
                    v-for="recording in filteredRecordings"
                    :key="recording.id"
                    class="recording-item p-2 mb-2 border rounded"
                    :class="{ 'bg-primary text-white': selectedRecording?.id === recording.id }"
                    @click="selectRecording(recording.id)"
                    style="cursor: pointer;"
                  >
                    <div class="fw-bold">{{ recording.name }}</div>
                    <small>{{ formatDate(recording.created_at) }}</small>
                    <div><small>{{ recording.waypoint_count }} waypoints</small></div>
                  </div>
                </div>
                <div class="flex-fill d-flex flex-column gap-2">
                  <div
                    ref="mapContainer"
                    class="border rounded flex-fill"
                    style="min-height: 0;"
                  ></div>
                  <div v-if="selectedRecording && waypoints.length > 0" class="border rounded p-2">
                    <div class="d-flex align-items-center gap-2 mb-2">
                      <button
                        class="btn btn-sm"
                        :class="isPlaying ? 'btn-danger' : 'btn-success'"
                        @click="togglePlayback"
                      >
                        {{ isPlaying ? 'Pause' : 'Play' }}
                      </button>
                      <button class="btn btn-sm btn-secondary" @click="resetPlayback">
                        Reset
                      </button>
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
  </Transition>
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
      if (playbackInterval !== null) {
        clearInterval(playbackInterval)
        playbackInterval = null
      }
      initializeMap()
    }
  } catch (error) {
    console.error('Failed to load waypoints:', error)
  }
}

const initializeMap = async () => {
  await nextTick()
  if (!mapContainer.value || waypoints.value.length === 0) return

  if (map) {
    map.remove()
  }

  map = L.map(mapContainer.value).setView(
    [waypoints.value[0].lat, waypoints.value[0].lon],
    18
  )

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
  const latLng = L.latLng(waypoint.lat, waypoint.lon)
  marker.setLatLng(latLng)
  map.panTo(latLng)
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
        if (playbackInterval !== null) {
          clearInterval(playbackInterval)
          playbackInterval = null
        }
      }
    }, 100)
  } else {
    if (playbackInterval !== null) {
      clearInterval(playbackInterval)
      playbackInterval = null
    }
  }
}

const resetPlayback = () => {
  currentWaypointIndex.value = 0
  isPlaying.value = false
  if (playbackInterval !== null) {
    clearInterval(playbackInterval)
    playbackInterval = null
  }
  updateMapPosition()
}

const formatDate = (dateString: string) => {
  return new Date(dateString).toLocaleString()
}

watch(() => props.show, async (newVal) => {
  if (newVal) {
    await loadRecordings()
  } else {
    if (playbackInterval !== null) {
      clearInterval(playbackInterval)
      playbackInterval = null
    }
    isPlaying.value = false
  }
})

watch(activeTab, () => {
  selectedRecording.value = null
  waypoints.value = []
  currentWaypointIndex.value = 0
  isPlaying.value = false
  if (playbackInterval !== null) {
    clearInterval(playbackInterval)
    playbackInterval = null
  }
  if (map) {
    map.remove()
    map = null
  }
})

onBeforeUnmount(() => {
  if (playbackInterval !== null) {
    clearInterval(playbackInterval)
  }
  if (map) {
    map.remove()
  }
})
</script>

<style scoped>
.modal-backdrop {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-color: rgba(0, 0, 0, 0.5);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 1050;
}

.modal-dialog {
  max-width: 1200px;
  width: 90%;
}

.modal-content {
  background-color: white;
  border-radius: 8px;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
}

.modal-header {
  padding: 1rem;
  border-bottom: 1px solid #dee2e6;
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.recording-item:hover {
  background-color: #f8f9fa;
}

.recording-item.bg-primary:hover {
  background-color: #0d6efd !important;
}

.modal-enter-active,
.modal-leave-active {
  transition: opacity 0.2s ease;
}

.modal-enter-active .modal-dialog,
.modal-leave-active .modal-dialog {
  transition: transform 0.2s ease, opacity 0.2s ease;
}

.modal-enter-from,
.modal-leave-to {
  opacity: 0;
}

.modal-enter-from .modal-dialog,
.modal-leave-to .modal-dialog {
  transform: scale(0.95);
  opacity: 0;
}
</style>

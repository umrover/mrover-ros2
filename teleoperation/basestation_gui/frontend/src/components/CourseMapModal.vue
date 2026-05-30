<template>
  <Teleport to="body">
    <div v-if="show" class="modal-backdrop" @click.self="$emit('close')">
      <div class="modal-dialog course-map-dialog">
        <div class="modal-content h-full flex flex-col overflow-hidden">
          <div class="modal-header">
            <h5 class="modal-title">Course Map</h5>
            <div class="flex items-center gap-2">
              <button
                class="btn btn-sm btn-primary"
                :disabled="scienceStore.waypoints.length === 0 || isDownloading"
                @click="downloadMapPNG"
              >
                <i class="bi bi-download"></i> PNG
              </button>
              <button type="button" class="btn-close" @click="$emit('close')">
                <i class="bi bi-x-lg"></i>
              </button>
            </div>
          </div>

          <div class="modal-body p-3 flex flex-col flex-1 min-h-0">
            <div
              v-if="scienceStore.waypoints.length === 0"
              class="flex items-center justify-center h-full"
            >
              <div class="alert alert-warning mb-0" role="alert">
                <h5 class="alert-heading font-bold">No Waypoints</h5>
                <p class="mb-0 text-sm">Add waypoints to the current course first.</p>
              </div>
            </div>

            <div v-else ref="mapCaptureRef" class="flex-1 rounded overflow-hidden border relative">
              <l-map
                ref="mapRef"
                class="w-full h-full"
                :zoom="19"
                :center="mapCenter"
                :options="{ scrollWheelZoom: true }"
              >
                <l-tile-layer :url="onlineUrl" :options="onlineTileOptions" />

                <l-polyline :lat-lngs="pathLatLngs" :color="'var(--accent)'" :weight="3" :opacity="0.7" />

                <l-marker
                  v-for="(wp, i) in scienceStore.waypoints"
                  :key="wp.db_id ?? i"
                  :lat-lng="[wp.lat, wp.lon]"
                  :icon="waypointIcon"
                >
                  <l-tooltip :permanent="true" :direction="'top'" :offset="[0, -36]">
                    {{ wp.name }}
                  </l-tooltip>
                </l-marker>

                <l-marker :lat-lng="startLatLng" :icon="startIcon">
                  <l-tooltip>Start</l-tooltip>
                </l-marker>
                <l-marker :lat-lng="endLatLng" :icon="endIcon">
                  <l-tooltip>End</l-tooltip>
                </l-marker>
              </l-map>
            </div>
          </div>
        </div>
      </div>
    </div>
  </Teleport>
</template>

<script lang="ts" setup>
import { ref, computed, watch, nextTick } from 'vue'
import { LMap, LTileLayer, LMarker, LPolyline, LTooltip } from '@vue-leaflet/vue-leaflet'
import 'leaflet/dist/leaflet.css'
import L from 'leaflet'
import 'leaflet-rotatedmarker'
import { toPng } from 'html-to-image'
import { useScienceWaypointStore } from '@/stores/scienceWaypoints'
import { useRoverMap } from '@/composables/useRoverMap'
import { currentTimestamp } from '@/utils/formatNumber'

const props = defineProps<{ show: boolean }>()
defineEmits(['close'])

const scienceStore = useScienceWaypointStore()
const { mapRef, onlineUrl, onlineTileOptions, getMap } = useRoverMap()

const mapCaptureRef = ref<HTMLElement | null>(null)
const isDownloading = ref(false)

const waypointIcon = L.icon({
  iconUrl: '/waypoint_marker.svg',
  iconSize: [40, 40],
  iconAnchor: [20, 40],
  popupAnchor: [0, -20],
})

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

const mapCenter = computed<[number, number]>(() => {
  const first = scienceStore.waypoints[0]
  if (!first) return [0, 0]
  return [first.lat, first.lon]
})

const pathLatLngs = computed<[number, number][]>(() =>
  scienceStore.waypoints.map(wp => [wp.lat, wp.lon])
)

const startLatLng = computed<[number, number]>(() => {
  const first = scienceStore.waypoints[0]
  return first ? [first.lat, first.lon] : [0, 0]
})

const endLatLng = computed<[number, number]>(() => {
  const last = scienceStore.waypoints[scienceStore.waypoints.length - 1]
  return last ? [last.lat, last.lon] : [0, 0]
})

watch(
  () => props.show,
  async (visible) => {
    if (!visible || scienceStore.waypoints.length === 0) return
    await nextTick()
    const map = getMap()
    if (map) {
      const bounds = L.latLngBounds(scienceStore.waypoints.map(wp => [wp.lat, wp.lon]))
      map.fitBounds(bounds, { padding: [40, 40], maxZoom: 19 })
    }
  }
)

async function downloadMapPNG() {
  if (!mapCaptureRef.value) return
  isDownloading.value = true
  try {
    await nextTick()
    const dataUrl = await toPng(mapCaptureRef.value, {
      cacheBust: true,
      backgroundColor: '#dddddd',
    })
    const link = document.createElement('a')
    link.download = `mission_course_map_${currentTimestamp()}.png`
    link.href = dataUrl
    link.click()
  } catch (err) {
    console.error('Failed to capture map image:', err)
  } finally {
    isDownloading.value = false
  }
}
</script>

<script lang="ts">
export default { name: 'CourseMapModal' }
</script>

<style scoped>
.course-map-dialog {
  max-width: 900px;
  width: 85%;
  height: 75vh;
}

.btn-close {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  width: 1.75rem;
  height: 1.75rem;
  padding: 0;
  font-size: 0.75rem;
  background: none;
  border: none;
  cursor: pointer;
  color: var(--text-muted);
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
</style>

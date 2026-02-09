<template>
  <div class="position-relative w-100 h-100 map">
    <l-map
      @ready="handleMapReady"
      ref="mapRef"
      class="map z-0"
      :zoom="22"
      :center="center"
      @click="getClickedLatLon($event)"
    >
      <l-control-scale :imperial="false" />
      <l-tile-layer
        ref="tileLayer"
        :url="online ? onlineUrl : offlineUrl"
        :attribution="attribution"
        :options="online ? onlineTileOptions : offlineTileOptions"
      />

      <l-marker ref="roverRef" :lat-lng="odomLatLng" :icon="locationIcon" />
      <l-marker ref="droneRef" :lat-lng="droneLatLng" :icon="droneIcon" />

      <div v-for="(waypoint, index) in waypointListForMap" :key="index">
        <l-marker
          :lat-lng="waypoint.latLng"
          :icon="getWaypointIcon(waypoint, index)"
        >
          <l-tooltip :options="{ permanent: 'true', direction: 'top' }">
            {{ waypoint.name }}, {{ index }}
          </l-tooltip>
        </l-marker>
      </div>

      <l-polyline :lat-lngs="odomPath" :color="'blue'" />
      <l-polyline :lat-lngs="dronePath" :color="'green'" />
    </l-map>
    <div class="map-controls cmd-panel">
      <div class="d-flex align-items-center gap-2">
        <input
          v-model="online"
          type="checkbox"
          class="form-check-input p-0"
        />
        <span class="cmd-data-label">Online</span>
      </div>
      <button @click="centerOnRover" class="btn btn-sm btn-outline-control border-2 map-btn">
        Center
      </button>
    </div>
  </div>
</template>

<script lang="ts" setup>
import {
  LMap,
  LTileLayer,
  LMarker,
  LPolyline,
  LTooltip,
  LControlScale,
} from '@vue-leaflet/vue-leaflet'
import { useErdStore } from '@/stores/erd'
import { storeToRefs } from 'pinia'
import 'leaflet/dist/leaflet.css'
import L from 'leaflet'
import 'leaflet-rotatedmarker'
import type { LeafletMouseEvent } from 'leaflet'
import type { MapWaypoint } from '@/types/waypoints'
import type { NavMessage } from '@/types/coordinates'
import { ref, shallowRef, triggerRef, computed, watch } from 'vue'
import { useRoverMap } from '@/composables/useRoverMap'

const erdStore = useErdStore()
const { waypointListForMap, highlightedWaypoint, searchWaypoint } = storeToRefs(erdStore)

const {
  center,
  online,
  mapRef,
  roverRef,
  odomPath,
  odomLatLng,
  onlineUrl,
  offlineUrl,
  onlineTileOptions,
  offlineTileOptions,
  attribution,
  locationIcon,
  waypointIcon,
  onMapReady,
  centerOnRover,
  getMap,
  navMessage,
} = useRoverMap({
  maxOdomCount: 1000,
  drawFrequency: 1,
  initialCenter: [38.4225202, -110.7844653],
})

const drone_latitude_deg = ref(0)
const drone_longitude_deg = ref(0)
const droneRef = ref<{ leafletObject: L.Marker } | null>(null)
let droneMarker: L.Marker | null = null
const dronePath = shallowRef<L.LatLng[]>([])
const circle = ref<L.Circle | null>(null)

const droneIcon = L.icon({
  iconUrl: '/drone_marker.svg',
  iconSize: [64, 64],
  iconAnchor: [32, 32],
})
const droneWaypointIcon = L.icon({
  iconUrl: '/waypoint_marker_drone.svg',
  iconSize: [64, 64],
  iconAnchor: [32, 64],
  popupAnchor: [0, -32],
})
const highlightedWaypointIcon = L.icon({
  iconUrl: '/waypoint_marker_highlighted.svg',
  iconSize: [64, 64],
  iconAnchor: [32, 64],
  popupAnchor: [0, -32],
})

const droneLatLng = computed(() => {
  return L.latLng(drone_latitude_deg.value, drone_longitude_deg.value)
})

const handleMapReady = () => {
  onMapReady(() => {
    if (droneRef.value) {
      droneMarker = droneRef.value.leafletObject as L.Marker
    }
  })
}

const getClickedLatLon = (e: LeafletMouseEvent) => {
  erdStore.clickPoint = {
    lat: e.latlng.lat,
    lon: e.latlng.lng,
  }
}

const getWaypointIcon = (waypoint: MapWaypoint, index: number) => {
  if (index === highlightedWaypoint.value) {
    return highlightedWaypointIcon
  } else if (waypoint.drone) {
    return droneWaypointIcon
  } else {
    return waypointIcon
  }
}

watch(navMessage, (msg) => {
  if (!msg) return
  const navMsg = msg as NavMessage
  if (navMsg.type === 'drone_waypoint') {
    drone_latitude_deg.value = navMsg.latitude
    drone_longitude_deg.value = navMsg.longitude
  }
})

watch([drone_latitude_deg, drone_longitude_deg], () => {
  const latLng = L.latLng(drone_latitude_deg.value, drone_longitude_deg.value)
  if (droneMarker) {
    droneMarker.setLatLng(latLng)
  }

  if (dronePath.value.length > 1000) {
    dronePath.value.shift()
  }
  dronePath.value.push(latLng)
  triggerRef(dronePath)
})

watch(searchWaypoint, (newIndex) => {
  const map = getMap()
  if (newIndex === -1) {
    if (map && circle.value) {
      map.removeLayer(circle.value as unknown as L.Layer)
    }
    circle.value = null
    return
  }
  const waypoint = waypointListForMap.value[newIndex]
  if (!waypoint) return

  if (!circle.value) {
    if (map) {
      circle.value = L.circle(waypoint.latLng, { radius: 20 }).addTo(map)
    }
  } else {
    circle.value.setLatLng(waypoint.latLng)
  }
  if (circle.value) {
    circle.value.setStyle({ fillColor: 'purple', stroke: false })
  }
})
</script>

<style scoped>
.map {
  min-height: 50vh;
}

.map-controls {
  position: absolute;
  top: var(--cmd-gap-md);
  right: var(--cmd-gap-md);
  display: flex;
  flex-direction: column;
  gap: var(--cmd-gap-sm);
  z-index: 1000;
}

.map-btn {
  font-size: var(--cmd-font-xs);
  text-transform: uppercase;
}

</style>

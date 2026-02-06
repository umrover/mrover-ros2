<template>
  <div class="position-relative w-100 h-100">
    <l-map
      @ready="handleMapReady"
      ref="mapRef"
      class="map z-0"
      :zoom="16"
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
      <l-marker
        ref="basestationRef"
        :lat-lng="basestationLatLng"
        :icon="basestationIcon"
      />
      <l-marker
        v-for="(waypoint, index) in waypointList"
        :key="index"
        :lat-lng="waypoint.latLng"
        :icon="waypointIcon"
      >
        <l-tooltip :options="{ permanent: 'true', direction: 'top' }">
          {{ waypoint.name }}
        </l-tooltip>
      </l-marker>
      <l-polyline
        :lat-lngs="polylinePath"
        :color="'red'"
        :dash-array="'5, 5'"
      />
      <l-polyline :lat-lngs="odomPath" :color="'blue'" :dash-array="'5, 5'" />
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
        Center on Rover
      </button>
      <button @click="centerOnBasestation" class="btn btn-sm btn-outline-control border-2 map-btn">
        Center on Base
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
import { useAutonomyStore } from '@/stores/autonomy'
import { storeToRefs } from 'pinia'
import 'leaflet/dist/leaflet.css'
import L from 'leaflet'
import { ref, computed, watch } from 'vue'
import { useRoverMap } from '@/composables/useRoverMap'
import type { NavMessage } from '@/types/coordinates'

const autonomyStore = useAutonomyStore()
const { route, waypointList } = storeToRefs(autonomyStore)
const { setClickPoint } = autonomyStore

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
  maxOdomCount: 10,
  drawFrequency: 10,
  initialCenter: [38.4071654, -110.7923927],
  offlineUrl: 'map/urc/{z}/{x}/{y}.jpg',
})

const basestation_latitude_deg = ref(0)
const basestation_longitude_deg = ref(0)

const basestationIcon = L.icon({
  iconUrl: '/basestation_marker.svg',
  iconSize: [64, 64],
  iconAnchor: [32, 56],
})

const basestationLatLng = computed(() => {
  return L.latLng(basestation_latitude_deg.value, basestation_longitude_deg.value)
})

const polylinePath = computed(() => {
  return [odomLatLng.value].concat(
    route.value.map((waypoint) => waypoint.latLng),
  )
})

const handleMapReady = () => {
  onMapReady()
}

const centerOnBasestation = () => {
  const map = getMap()
  if (map) {
    map.setView(basestationLatLng.value, map.getZoom())
  }
}

const getClickedLatLon = (e: { latlng: { lat: number; lng: number } }) => {
  setClickPoint({
    lat: e.latlng.lat,
    lon: e.latlng.lng,
  })
}

watch(navMessage, (msg) => {
  if (!msg) return
  const navMsg = msg as NavMessage
  if (navMsg.type === 'basestation_position') {
    basestation_latitude_deg.value = navMsg.latitude
    basestation_longitude_deg.value = navMsg.longitude
  }
})
</script>

<style scoped>
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

<template>
  <div class="relative w-full h-full">
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
        :key="online ? 'online' : 'offline'"
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
      <l-marker ref="droneRef" :lat-lng="droneLatLng" :icon="droneIcon" />
      <l-marker
        v-for="(waypoint, index) in storeForMap"
        :key="index"
        :lat-lng="waypoint.latLng"
        :icon="waypointIcon"
      >
        <l-tooltip :options="{ permanent: 'true', direction: 'top' }">
          {{ waypoint.name }}
        </l-tooltip>
      </l-marker>
      <l-polyline
        :lat-lngs="executionPath"
        color="red"
        dash-array="5, 5"
      />
      <l-polyline :lat-lngs="odomPath" :color="'blue'" :dash-array="'5, 5'" />
      <l-polyline :lat-lngs="dronePath" :color="'green'" />
    </l-map>

    <div class="overlay-toolbar right-0">
      <button class="overlay-toolbar-btn" @click="online = !online">
        Online <i :class="online ? 'bi bi-check-square-fill' : 'bi bi-square'"></i>
      </button>
      <button class="overlay-toolbar-btn" @click="followRover = !followRover">
        Follow Rover <i :class="followRover ? 'bi bi-check-square-fill' : 'bi bi-square'"></i>
      </button>
      <div class="overlay-dropdown">
        <button class="overlay-toolbar-btn" :disabled="followRover" @click="centerOpen = !centerOpen; zoomOpen = false">
          Center <i class="bi bi-chevron-down"></i>
        </button>
        <div v-if="centerOpen" class="overlay-dropdown-menu">
          <button class="overlay-dropdown-item" @click="centerOnRover(); centerOpen = false">Rover</button>
          <button class="overlay-dropdown-item" @click="centerOnBasestation(); centerOpen = false">Base</button>
          <button class="overlay-dropdown-item" @click="centerOnDrone(); centerOpen = false">Drone</button>
        </div>
      </div>
      <div class="overlay-dropdown">
        <button class="overlay-toolbar-btn" @click="zoomOpen = !zoomOpen; centerOpen = false">
          Zoom <i class="bi bi-chevron-down"></i>
        </button>
        <div v-if="zoomOpen" class="overlay-dropdown-menu">
          <button
            v-for="z in zoomLevels"
            :key="z"
            class="overlay-dropdown-item"
            @click="setZoom(z); zoomOpen = false"
          >
            {{ z }}
          </button>
        </div>
      </div>
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
import { ref, computed } from 'vue'
import { useRoverMap } from '@/composables/useRoverMap'
import { useWebsocketStore } from '@/stores/websocket'
import type { BasestationPositionMessage } from '@/types/coordinates'

const autonomyStore = useAutonomyStore()
const { executionForMap, storeForMap } = storeToRefs(autonomyStore)

const {
  center,
  online,
  followRover,
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
  droneRef,
  dronePath,
  droneLatLng,
  droneIcon,
  onMapReady,
  centerOnRover,
  centerOnDrone,
  centerOpen,
  zoomOpen,
  zoomLevels,
  setZoom,
  getMap,
} = useRoverMap({
  maxOdomCount: 500,
  drawFrequency: 1,
  initialCenter: [38.4071654, -110.7923927],
  offlineUrl: '/map/{z}/{x}/{y}.png',
})

const websocketStore = useWebsocketStore()

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

const executionPath = computed(() => {
  return [odomLatLng.value].concat(
    executionForMap.value.map((wp) => wp.latLng),
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
  autonomyStore.clickPoint = {
    lat: e.latlng.lat,
    lon: e.latlng.lng,
  }
}

websocketStore.onMessage<BasestationPositionMessage>('nav', 'basestation_position', (msg) => {
  basestation_latitude_deg.value = msg.latitude
  basestation_longitude_deg.value = msg.longitude
})
</script>

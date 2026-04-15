<template>
  <div class="relative w-full h-full">
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
      <l-marker
        v-for="(waypoint, index) in waypointListForMap"
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

    <div class="overlay-toolbar right-0">
      <button class="overlay-toolbar-btn" :class="{ 'overlay-toolbar-btn-active': online }" @click="online = !online">
        Online
      </button>
      <button @click="centerOnRover" class="overlay-toolbar-btn">Center on Rover</button>
      <button @click="centerOnBasestation" class="overlay-toolbar-btn">Center on Base</button>
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
const { routeForMap, waypointListForMap } = storeToRefs(autonomyStore)

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
} = useRoverMap({
  maxOdomCount: 100,
  drawFrequency: 1,
  initialCenter: [38.4071654, -110.7923927],
  offlineUrl: '/map/{z}/{x}/{y}.png',
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
    routeForMap.value.map((waypoint) => waypoint.latLng),
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

const websocketStore = useWebsocketStore()
websocketStore.onMessage<BasestationPositionMessage>('nav', 'basestation_position', (msg) => {
  basestation_latitude_deg.value = msg.latitude
  basestation_longitude_deg.value = msg.longitude
})
</script>

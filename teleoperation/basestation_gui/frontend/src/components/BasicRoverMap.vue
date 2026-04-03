<template>
  <div class="relative w-full h-full map">
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
import { useErdStore } from '@/stores/erd'
import { storeToRefs } from 'pinia'
import 'leaflet/dist/leaflet.css'
import L from 'leaflet'
import 'leaflet-rotatedmarker'
import type { LeafletMouseEvent } from 'leaflet'
import type { MapWaypoint } from '@/types/waypoints'
import { ref, watch } from 'vue'
import { useRoverMap } from '@/composables/useRoverMap'

const erdStore = useErdStore()
const { waypointListForMap, highlightedWaypoint, searchWaypoint } = storeToRefs(erdStore)

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
  maxOdomCount: 1000,
  drawFrequency: 1,
  initialCenter: [38.4225202, -110.7844653],
})

const circle = ref<L.Circle | null>(null)
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

const handleMapReady = () => {
  onMapReady()
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
</style>

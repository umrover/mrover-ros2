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

      <div v-for="(waypoint, index) in waypointList" :key="index">
        <l-marker
          :lat-lng="waypoint.latLng"
          :icon="getWaypointIcon(waypoint, index)"
        >
          <l-tooltip :options="{ permanent: 'true', direction: 'top' }">
            {{ waypoint.name }}, {{ index }}
          </l-tooltip>
        </l-marker>
      </div>

      <l-polyline :lat-lngs="[...odomPath]" :color="'blue'" />
      <l-polyline :lat-lngs="[...dronePath]" :color="'green'" />
    </l-map>
    <div class="controls px-2 py-2 position-absolute d-flex flex-column gap-2 top-0 end-0 m-2 rounded border shadow-sm" style="background-color: rgba(255, 255, 255, 0.9)">
      <div class="d-flex align-items-center gap-2">
        <input
          v-model="online"
          type="checkbox"
          class="form-check-input p-0"
        />
        <p class="mb-0 text-body" style="font-size: 14px; line-height: 18px">
          Online
        </p>
      </div>
      <button @click="centerOnRover" class="btn btn-sm btn-light border" style="font-size: 14px; padding: 4px 8px">
        Center
      </button>
    </div>

    <div class="odometry" v-if="odom">
      <p>
        Lat: {{ odom.latitude_deg.toFixed(6) }} N, Lon:
        {{ odom.longitude_deg.toFixed(6) }} E
      </p>
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
import type { StoreWaypoint } from '@/types/waypoints'
import type { Odom, NavMessage } from '@/types/coordinates'
import { ref, computed, watch } from 'vue'
import { useRoverMap } from '@/composables/useRoverMap'

const erdStore = useErdStore()
const { waypointList, highlightedWaypoint, searchWaypoint } = storeToRefs(erdStore)
const { setClickPoint } = erdStore

const {
  center,
  online,
  mapRef,
  roverRef,
  odomPath,
  odomLatLng,
  rover_latitude_deg,
  rover_longitude_deg,
  rover_bearing_deg,
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
const droneCount = ref(0)
const dronePath = ref<L.LatLng[]>([])
const circle = ref<L.Circle | null>(null)

const odom = computed<Odom>(() => ({
  latitude_deg: rover_latitude_deg.value,
  longitude_deg: rover_longitude_deg.value,
  bearing_deg: rover_bearing_deg.value,
}))

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
  console.log(e)
  setClickPoint({
    lat: e.latlng.lat,
    lon: e.latlng.lng,
  })
}

const getWaypointIcon = (waypoint: StoreWaypoint, index: number) => {
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

  droneCount.value++
  if (droneCount.value % 1 === 0) {
    if (dronePath.value.length > 1000) {
      dronePath.value = [...dronePath.value.slice(1), latLng]
    } else {
      dronePath.value = [...dronePath.value, latLng]
    }
    droneCount.value = 0
  }
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
  const waypoint = waypointList.value[newIndex]
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

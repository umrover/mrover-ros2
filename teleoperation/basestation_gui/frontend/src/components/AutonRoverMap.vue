<template>
  <div class="position-relative w-100 h-100">
    <l-map
      @ready="onMapReady"
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
      <l-polyline :lat-lngs="[...odomPath]" :color="'blue'" :dash-array="'5, 5'" />
    </l-map>

    <!-- Controls -->
    <div class="controls px-2 py-1 position-absolute d-flex align-items-center gap-2 top-0 end-0 m-2 bg-white rounded">
      <input
        v-model="online"
        type="checkbox"
        class="form-check-input p-0"
      />
      <p class="mb-0 text-body" style="font-size: 14px; line-height: 18px">
        Online
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
import { useAutonomyStore } from '@/stores/autonomy'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import 'leaflet/dist/leaflet.css'
import L from 'leaflet'
import '../leaflet-rotatedmarker'
import { ref, computed, watch, nextTick } from 'vue'
import { quaternionToMapAngle } from '../utils/map'
import type { NavMessage } from '../types/coordinates'

const autonomyStore = useAutonomyStore()
const { route, waypointList } = storeToRefs(autonomyStore)
const { setClickPoint } = autonomyStore

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const rover_latitude_deg = ref(0)
const rover_longitude_deg = ref(0)
const rover_bearing_deg = ref(0)
const basestation_latitude_deg = ref(0)
const basestation_longitude_deg = ref(0)

const MAX_ODOM_COUNT = 10
const DRAW_FREQUENCY = 10
const onlineUrl = 'http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}'
const offlineUrl = 'map/urc/{z}/{x}/{y}.jpg'
const onlineTileOptions = {
  maxNativeZoom: 22,
  maxZoom: 100,
  subdomains: ['mt0', 'mt1', 'mt2', 'mt3'],
}
const offlineTileOptions = {
  minZoom: 16,
  maxZoom: 20,
}

const center = ref<[number, number]>([38.4071654, -110.7923927])
const attribution = ref('&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors')
const online = ref(true)
const mapRef = ref<{ leafletObject: L.Map } | null>(null)
const roverRef = ref<{ leafletObject: L.Marker } | null>(null)
const basestationRef = ref(null)
let roverMarker: L.Marker | null = null
const odomCount = ref(0)
const odomPath = ref<L.LatLng[]>([])
const findRover = ref(false)

const locationIcon = L.icon({
  iconUrl: 'location_marker_icon.png',
  iconSize: [40, 40],
  iconAnchor: [20, 20],
})
const basestationIcon = L.icon({
  iconUrl: 'basestation_marker_icon.png',
  iconSize: [40, 40],
  iconAnchor: [20, 20],
})
const waypointIcon = L.icon({
  iconUrl: 'map_marker.png',
  iconSize: [64, 64],
  iconAnchor: [32, 64],
  popupAnchor: [0, -32],
})

const onMapReady = () => {
  nextTick(() => {
    if (roverRef.value) {
      roverMarker = roverRef.value.leafletObject as L.Marker
    }
  })
}

const getClickedLatLon = (e: { latlng: { lat: number; lng: number } }) => {
  setClickPoint({
    lat: e.latlng.lat,
    lon: e.latlng.lng,
  })
}

const odomLatLng = computed(() => {
  return L.latLng(rover_latitude_deg.value, rover_longitude_deg.value)
})

const basestationLatLng = computed(() => {
  return L.latLng(basestation_latitude_deg.value, basestation_longitude_deg.value)
})

const polylinePath = computed(() => {
  return [odomLatLng.value].concat(
    route.value.map((waypoint) => waypoint.latLng),
  )
})

const navMessage = computed(() => messages.value['nav'])

watch(navMessage, (msg) => {
  if (!msg) return
  const navMsg = msg as NavMessage

  if (navMsg.type === 'gps_fix') {
    rover_latitude_deg.value = navMsg.latitude
    rover_longitude_deg.value = navMsg.longitude
  } else if (navMsg.type === 'basestation_position') {
    basestation_latitude_deg.value = navMsg.latitude
    basestation_longitude_deg.value = navMsg.longitude
  } else if (navMsg.type === 'orientation') {
    rover_bearing_deg.value = quaternionToMapAngle(navMsg.orientation)
  }
})

watch([rover_latitude_deg, rover_longitude_deg, rover_bearing_deg], () => {
  const lat = rover_latitude_deg.value
  const lng = rover_longitude_deg.value
  const angle = rover_bearing_deg.value

  const latLng = L.latLng(lat, lng)

  if (!findRover.value) {
    findRover.value = true
    center.value = [lat, lng]
  }

  if (roverMarker !== null) {
    roverMarker.setRotationAngle(angle)
    roverMarker.setLatLng(latLng)
  }

  odomCount.value++
  if (odomCount.value % DRAW_FREQUENCY === 0) {
    if (odomPath.value.length > MAX_ODOM_COUNT) {
      odomPath.value = [...odomPath.value.slice(1), latLng]
    } else {
      odomPath.value = [...odomPath.value, latLng]
    }
    odomCount.value = 0
  }
})
</script>
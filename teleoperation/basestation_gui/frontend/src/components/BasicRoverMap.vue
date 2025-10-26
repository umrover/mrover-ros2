<template>
  <div class="position-relative w-100 h-100 map">
    <l-map
      @ready="onMapReady"
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
    <div
      class="controls px-2 py-1 position-absolute d-flex align-items-center gap-2 top-0 end-0 m-2 bg-white rounded"
    >
      <input
        v-model="online"
        type="checkbox"
        class="form-check-input p-0"
        style="width: 14px; height: 14px; vertical-align: middle"
      />
      <p class="mb-0 text-body" style="font-size: 14px; line-height: 18px">
        Online
      </p>
    </div>

    <div class="odometry" v-if="odom">
      <p>
        Lat: {{ odom.latitude_deg.toFixed(6) }}º N, Lon:
        {{ odom.longitude_deg.toFixed(6) }}º E
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
import '../leaflet-rotatedmarker.js'
import type { LeafletMouseEvent } from 'leaflet';
import type { StoreWaypoint } from '@/types/waypoints'
import type { Odom } from '@/types/coordinates'
import { ref, computed, watch, nextTick } from 'vue'
import type { PropType } from 'vue'

const props = defineProps({
  odom: {
    type: Object as PropType<Odom | null>,
    default: () => ({ latitude_deg: 0, longitude_deg: 0, bearing_deg: 0 }),
  },
  drone_odom: {
    type: Object,
    default: () => ({ latitude_deg: 0, longitude_deg: 0, bearing_deg: 0 }),
  },
})

const erdStore = useErdStore()
const { waypointList, highlightedWaypoint, searchWaypoint } = storeToRefs(erdStore)
const { setClickPoint } = erdStore

const MAX_ODOM_COUNT = 1000
const DRAW_FREQUENCY = 1
const onlineUrl = 'http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}'
const offlineUrl = 'map/{z}/{x}/{y}.png'
const onlineTileOptions = {
  maxNativeZoom: 22,
  maxZoom: 100,
  subdomains: ['mt0', 'mt1', 'mt2', 'mt3'],
}
const offlineTileOptions = {
  maxNativeZoom: 16,
  maxZoom: 100,
}

const center = ref<[number, number]>([38.4225202, -110.7844653])
const attribution = ref('&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors')
const online = ref(true)
const mapRef = ref<{ leafletObject: L.Map } | null>(null)
const roverRef = ref<{ leafletObject: L.Marker } | null>(null)
const droneRef = ref<{ leafletObject: L.Marker } | null>(null)
let map: L.Map | null = null
let roverMarker: L.Marker | null = null
let droneMarker: L.Marker | null = null
const odomCount = ref(0)
const droneCount = ref(0)
const odomPath = ref<L.LatLng[]>([])
const dronePath = ref<L.LatLng[]>([])
const findRover = ref(false)
const circle = ref<L.Circle | null>(null)

const locationIcon = L.icon({
  iconUrl: '/location_marker_icon.png',
  iconSize: [64, 64],
  iconAnchor: [32, 32],
})
const droneIcon = L.icon({
  iconUrl: '/drone_icon_1.png',
  iconSize: [64, 64],
  iconAnchor: [32, 32],
})
const waypointIcon = L.icon({
  iconUrl: '/map_marker.png',
  iconSize: [64, 64],
  iconAnchor: [32, 64],
  popupAnchor: [0, -32],
})
const droneWaypointIcon = L.icon({
  iconUrl: '/map_marker_drone.png',
  iconSize: [64, 64],
  iconAnchor: [32, 64],
  popupAnchor: [0, -32],
})
const highlightedWaypointIcon = L.icon({
  iconUrl: '/map_marker_highlighted.png',
  iconSize: [64, 64],
  iconAnchor: [32, 64],
  popupAnchor: [0, -32],
})

const onMapReady = () => {
  nextTick(() => {
    if (mapRef.value) {
      map = mapRef.value.leafletObject as L.Map
    }
    if (roverRef.value) {
      roverMarker = roverRef.value.leafletObject as L.Marker
    }
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

const odomLatLng = computed(() => {
  if (
    props.odom &&
    typeof props.odom === 'object' &&
    props.odom.latitude_deg !== undefined &&
    props.odom.longitude_deg !== undefined
  ) {
    return L.latLng(props.odom.latitude_deg, props.odom.longitude_deg)
  } else {
    console.warn('odom data not ready yet')
    return L.latLng(0, 0)
  }
})

const droneLatLng = computed(() => {
  if (
    props.drone_odom &&
    typeof props.drone_odom === 'object' &&
    props.drone_odom.latitude_deg !== undefined &&
    props.drone_odom.longitude_deg !== undefined
  ) {
    return L.latLng(
      props.drone_odom.latitude_deg,
      props.drone_odom.longitude_deg,
    )
  } else {
    console.warn('drone odom data not ready yet')
    return L.latLng(0, 0)
  }
})

watch(() => props.odom, (val) => {
  if (!val) return;
  const lat = val.latitude_deg
  const lng = val.longitude_deg
  const angle = val.bearing_deg ?? 0

  const latLng = L.latLng(lat, lng)

  if (!findRover.value) {
    findRover.value = true
    center.value = [lat, lng]
  }

  if (roverMarker) {
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
}, { deep: true })

watch(() => props.drone_odom, (val) => {
  const lat = val.latitude_deg
  const lng = val.longitude_deg

  const latLng = L.latLng(lat, lng)
  if (droneMarker) {
    droneMarker.setLatLng(latLng)
  }

  droneCount.value++
  if (droneCount.value % DRAW_FREQUENCY === 0) {
    if (dronePath.value.length > MAX_ODOM_COUNT) {
      dronePath.value = [...dronePath.value.slice(1), latLng]
    } else {
      dronePath.value = [...dronePath.value, latLng]
    }
    droneCount.value = 0
  }
}, { deep: true })

watch(searchWaypoint, (newIndex) => {
  if (newIndex === -1) {
    if (map && circle.value) {
      map.removeLayer(circle.value as unknown as L.Layer)
    }
    circle.value = null
    return
  }
  const waypoint = waypointList.value[newIndex]
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

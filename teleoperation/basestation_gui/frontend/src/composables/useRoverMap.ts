import L from 'leaflet'
import 'leaflet-rotatedmarker'
import { ref, computed, watch, nextTick } from 'vue'
import { useGridLayoutStore } from '@/stores/gridLayout'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import { quaternionToMapAngle } from '@/utils/map'
import type { NavMessage } from '@/types/coordinates'

export interface UseRoverMapOptions {
  maxOdomCount?: number
  drawFrequency?: number
  initialCenter?: [number, number]
  offlineUrl?: string
}

export function useRoverMap(options: UseRoverMapOptions = {}) {
  const {
    maxOdomCount = 100,
    drawFrequency = 1,
    initialCenter = [38.4071654, -110.7923927],
    offlineUrl = 'map/{z}/{x}/{y}.png',
  } = options

  const websocketStore = useWebsocketStore()
  const { messages } = storeToRefs(websocketStore)
  const gridLayoutStore = useGridLayoutStore()
  const { locked: gridLocked } = storeToRefs(gridLayoutStore)

  const rover_latitude_deg = ref(0)
  const rover_longitude_deg = ref(0)
  const rover_bearing_deg = ref(0)
  const center = ref<[number, number]>(initialCenter)
  const online = ref(true)
  const mapRef = ref<{ leafletObject: L.Map } | null>(null)
  const roverRef = ref<{ leafletObject: L.Marker } | null>(null)
  let roverMarker: L.Marker | null = null
  const odomCount = ref(0)
  const odomPath = ref<L.LatLng[]>([])
  const findRover = ref(false)

  const onlineUrl = 'http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}'
  const onlineTileOptions = {
    maxNativeZoom: 22,
    maxZoom: 100,
    subdomains: ['mt0', 'mt1', 'mt2', 'mt3'],
  }
  const offlineTileOptions = {
    maxNativeZoom: 16,
    maxZoom: 100,
  }
  const attribution = '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors'

  const locationIcon = L.icon({
    iconUrl: '/rover_marker.svg',
    iconSize: [64, 64],
    iconAnchor: [32, 32],
  })
  const waypointIcon = L.icon({
    iconUrl: '/waypoint_marker.svg',
    iconSize: [64, 64],
    iconAnchor: [32, 64],
    popupAnchor: [0, -32],
  })

  const odomLatLng = computed(() => L.latLng(rover_latitude_deg.value, rover_longitude_deg.value))
  const navMessage = computed(() => messages.value['nav'])

  const getMap = (): L.Map | null => {
    return mapRef.value?.leafletObject as L.Map | null
  }

  const onMapReady = (additionalSetup?: () => void) => {
    nextTick(() => {
      if (roverRef.value) {
        roverMarker = roverRef.value.leafletObject as L.Marker
      }
      const map = getMap()
      if (map && !gridLocked.value) {
        map.dragging.disable()
      }
      additionalSetup?.()
    })
  }

  const centerOnRover = () => {
    const map = getMap()
    if (map) {
      map.setView(odomLatLng.value, map.getZoom())
    }
  }

  watch(gridLocked, (locked) => {
    const map = getMap()
    if (!map) return
    if (locked) {
      map.dragging.enable()
    } else {
      map.dragging.disable()
    }
  }, { immediate: true })

  watch(navMessage, (msg) => {
    if (!msg) return
    const navMsg = msg as NavMessage
    if (navMsg.type === 'gps_fix') {
      rover_latitude_deg.value = navMsg.latitude
      rover_longitude_deg.value = navMsg.longitude
    } else if (navMsg.type === 'orientation') {
      rover_bearing_deg.value = quaternionToMapAngle(navMsg.orientation)
    }
  })

  watch([rover_latitude_deg, rover_longitude_deg, rover_bearing_deg], () => {
    const latLng = L.latLng(rover_latitude_deg.value, rover_longitude_deg.value)

    if (!findRover.value) {
      findRover.value = true
      center.value = [rover_latitude_deg.value, rover_longitude_deg.value]
    }

    if (roverMarker) {
      roverMarker.setRotationAngle(rover_bearing_deg.value)
      roverMarker.setLatLng(latLng)
    }

    odomCount.value++
    if (odomCount.value % drawFrequency === 0) {
      odomPath.value = odomPath.value.length > maxOdomCount
        ? [...odomPath.value.slice(1), latLng]
        : [...odomPath.value, latLng]
      odomCount.value = 0
    }
  })

  return {
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
  }
}

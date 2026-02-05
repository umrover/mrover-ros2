<template>
  <div class="position-relative w-100 h-100">
    <l-map
      @ready="onMapReady"
      ref="map"
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
      <l-marker ref="rover" :lat-lng="odomLatLng" :icon="locationIcon" />
      <l-marker
        ref="basestation"
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

<script lang="ts">
import {
  LMap,
  LTileLayer,
  LMarker,
  LPolyline,
  LTooltip,
  LControlScale,
} from '@vue-leaflet/vue-leaflet'
import Vuex from 'vuex'
const { mapGetters, mapMutations } = Vuex
import 'leaflet/dist/leaflet.css'
import L from '../leaflet-rotatedmarker'
import type { LatLng } from '@/types/leaflet'

const MAX_ODOM_COUNT = 10
const DRAW_FREQUENCY = 10
// Options for the tilelayer object on the map
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

export default {
  name: 'AutonRoverMap',
  components: {
    LMap,
    LTileLayer,
    LMarker,
    LPolyline,
    LTooltip,
    LControlScale,
  },
  props: {
    odom: {
      type: Object,
      default: () => ({ latitude_deg: 0, longitude_deg: 0, bearing_deg: 0 }),
    },
    basestation: {
      type: Object,
      default: () => ({ latitude_deg: 0, longitude_deg: 0 }),
    },
  },
  data() {
    return {
      center: L.latLng(38.4071654, -110.7923927),
      attribution:
        '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors',
      online: true,
      onlineUrl: onlineUrl,
      offlineUrl: offlineUrl,
      onlineTileOptions: onlineTileOptions,
      offlineTileOptions: offlineTileOptions,
      roverMarker: null,
      waypointIcon: null,
      locationIcon: null,

      map: null,
      odomCount: 0,
      odomPath: [],

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

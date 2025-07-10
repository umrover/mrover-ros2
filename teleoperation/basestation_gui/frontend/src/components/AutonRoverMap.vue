<template>
  <div class="wrap">
    <!-- Leaflet Map Definition-->
    <l-map @ready="onMapReady" ref="map" class="map" :zoom="16" :center="center" @click="getClickedLatLon($event)">
      <l-control-scale :imperial="false" />
      <!-- Tile Layer for map background -->
      <l-tile-layer ref="tileLayer" :url="online ? onlineUrl : offlineUrl" :attribution="attribution"
        :options="online ? onlineTileOptions : offlineTileOptions" />

      <!-- Markers for rover location -->
      <l-marker ref="rover" :lat-lng="odomLatLng" :icon="locationIcon" />

      <!-- Markers for basestation location -->
      <l-marker ref="basestation" :lat-lng="basestationLatLng" :icon="basestationIcon" />

      <!-- Waypoint Icons -->
      <l-marker v-for="(waypoint, index) in waypointList" :key="index" :lat-lng="waypoint.latLng" :icon="waypointIcon">
        <l-tooltip :options="{ permanent: 'true', direction: 'top' }">
          {{ waypoint.name }}
        </l-tooltip>
      </l-marker>

      <!-- Polylines -->
      <l-polyline :lat-lngs="polylinePath" :color="'red'" :dash-array="'5, 5'" />
      <l-polyline :lat-lngs="odomPath" :color="'blue'" :dash-array="'5, 5'" />
    </l-map>
    
    <!-- Controls -->
    <div class="controls px-2 py-1">
      <input v-model="online" type="checkbox" />
      <p>Online</p>
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
  LControlScale
} from '@vue-leaflet/vue-leaflet'
import Vuex from 'vuex'
const { mapGetters, mapMutations } = Vuex
import 'leaflet/dist/leaflet.css'
import L from '../leaflet-rotatedmarker'

const MAX_ODOM_COUNT = 10
const DRAW_FREQUENCY = 10
// Options for the tilelayer object on the map
const onlineUrl = 'http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}'
const offlineUrl = 'map/urc/{z}/{x}/{y}.jpg'
const onlineTileOptions = {
  maxNativeZoom: 22,
  maxZoom: 100,
  subdomains: ['mt0', 'mt1', 'mt2', 'mt3']
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
    LControlScale
  },
  props: {
    odom: {
      type: Object,
      default: () => ({latitude_deg: 0, longitude_deg: 0, bearing_deg: 0})
    },
    basestation: {
      type: Object,
      default: () => ({ latitude_deg: 0, longitude_deg: 0 })
    }
  },
  data() {
    return {
      center: L.latLng(38.4071654, -110.7923927),
      attribution: '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors',
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

      findRover: false
    }
  },
  computed: {
    ...mapGetters('autonomy', {
      route: 'route',
      waypointList: 'waypointList',
      autonEnabled: 'autonEnabled'
    }),

    // Convert to latLng object for Leaflet to use
    odomLatLng: function () {
      if (this.odom && typeof this.odom === 'object' && this.odom.latitude_deg !== undefined && this.odom.longitude_deg !== undefined ) {
        return L.latLng(this.odom.latitude_deg, this.odom.longitude_deg);
      } else {
        // Handle the case where odom is not yet ready
        console.warn('odom data not ready yet');
        return L.latLng(0,0); // or default value or return null
      }
    },

    basestationLatLng: function () {
      if (this.basestation && typeof this.basestation === 'object' && this.basestation.latitude_deg !== undefined && this.basestation.longitude_deg !== undefined ) {
        return L.latLng(this.basestation.latitude_deg, this.basestation.longitude_deg)
      } else {
        return L.latLng(0, 0)
      }
    },

    // Concat waypoints on course with rover marker at index 0 for polyline
    polylinePath: function () {
      return [this.odomLatLng].concat(
        this.route.map((waypoint: { latLng: any }) => waypoint.latLng)
      )
    }
  },
  watch: {
    odom: {
      handler: function (val) {
        // Trigger every time rover odom is changed
        const lat = val.latitude_deg
        const lng = val.longitude_deg
        const angle = val.bearing_deg

        const latLng = L.latLng(lat, lng)

        // Move to rover on first odom message
        if (!this.findRover) {
          this.findRover = true
          this.center = latLng
        }

        // Update the rover marker using bearing angle
        if (this.roverMarker !== null) {
          this.roverMarker.setRotationAngle(angle)
          this.roverMarker.setLatLng(latLng)
        }

        // Update the rover path
        this.odomCount++
        if (this.odomCount % DRAW_FREQUENCY === 0) {
          if (this.odomPath.length > MAX_ODOM_COUNT) {
            this.odomPath = [...this.odomPath.slice(1), latLng] //remove oldest element
          }

          this.odomPath = [...this.odomPath, latLng]
          this.odomCount = 0
        }
      },
      // Deep will watch for changes in children of an object
      deep: true
    }
  },
  created: function () {
    // Get Icons for Map
    this.locationIcon = L.icon({
      iconUrl: 'location_marker_icon.png',
      iconSize: [40, 40],
      iconAnchor: [20, 20]
    })
    this.basestationIcon = L.icon({
      iconUrl: 'basestation_marker_icon.png',
      iconSize: [40, 40],
      iconAnchor: [20, 20]
    })
    this.waypointIcon = L.icon({
      iconUrl: 'map_marker.png',
      iconSize: [64, 64],
      iconAnchor: [32, 64],
      popupAnchor: [0, -32]
    })
  },

  methods: {
    onMapReady: function () {
      // Pull objects from refs to be able to access data and change w functions
      this.$nextTick(() => {
        this.map = this.$refs.map.leafletObject
        this.roverMarker = this.$refs.rover.leafletObject
      })
    },
    // Event listener for setting store values to get data to waypoint Editor
    getClickedLatLon: function (e: { latlng: { lat: any; lng: any } }) {
      this.setClickPoint({
        lat: e.latlng.lat,
        lon: e.latlng.lng,
      })
    },

    ...mapMutations('autonomy', {
      setClickPoint: 'setClickPoint',
      setWaypointList: 'setWaypointList',
      setOdomFormat: 'setOdomFormat'
    })
  }
}
</script>

<style scoped>
.wrap {
  position: relative; /* Set this to position controls over map */
  width: 100%;
  height: 100%;
  min-height: 60vh;
}

.map {
  height: 100%;
  width: 100%;
}

.controls {
  position: absolute;
  display: flex;
  align-items: center;
  gap: 8px;
  top: 10px;
  right: 10px;
  background-color: rgba(255, 255, 255, 1);
  border-radius: 3px;
  z-index: 1000;
  font-size: 14px;
}

.controls input[type="checkbox"] {
  width: 14px;
  height: 14px;
  vertical-align: middle;
}

.controls p {
  margin: 0;
  font-size: 14px;
  color: #333;
  line-height: 18px;
}
</style>
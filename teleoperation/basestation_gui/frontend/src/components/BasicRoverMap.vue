<template>
  <div class="wrap">
      <l-map @ready="onMapReady" ref="map" class="map" :zoom="22" :center="center" @click="getClickedLatLon($event)">
          <l-control-scale :imperial="false" />
          <l-tile-layer ref="tileLayer" :url="online ? onlineUrl : offlineUrl" :attribution="attribution" :options="online ? onlineTileOptions : offlineTileOptions" />
  
          <l-marker ref="rover" :lat-lng="odomLatLng" :icon="locationIcon" />
          <l-marker ref="drone" :lat-lng="droneLatLng" :icon="droneIcon" />
  
          <div v-for="(waypoint, index) in waypointList" :key="index">
              <l-marker :lat-lng="waypoint.latLng" :icon="getWaypointIcon(waypoint, index)">
                  <l-tooltip :options="{ permanent: 'true', direction: 'top' }">
                      {{ waypoint.name }}, {{ index }}
                  </l-tooltip>
              </l-marker>
          </div>
  
          <l-polyline :lat-lngs="odomPath" :color="'blue'" />
          <l-polyline :lat-lngs="dronePath" :color="'green'" />
      </l-map>
      <label><input v-model="online" type="checkbox" /> Online</label>
        <div class="odometry">
            <p> Lat: {{ odom.latitude_deg.toFixed(6) }}º N, 
                Lon: {{ odom.longitude_deg.toFixed(6) }}º E
            </p>
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
  import {
      mapGetters,
      mapMutations,
      mapActions,
      mapState
  } from 'vuex'
  import 'leaflet/dist/leaflet.css'
  import L from '../leaflet-rotatedmarker.js'
  
  const MAX_ODOM_COUNT = 1000
  const DRAW_FREQUENCY = 1
  const onlineUrl = 'http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}'
  const offlineUrl = 'map/{z}/{x}/{y}.png'
  const onlineTileOptions = {
      maxNativeZoom: 22,
      maxZoom: 100,
      subdomains: ['mt0', 'mt1', 'mt2', 'mt3']
  }
  const offlineTileOptions = {
      maxNativeZoom: 16,
      maxZoom: 100
  }
  
  export default {
      name: 'RoverMap',
  
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
          drone_odom: {
              type: Object,
              default: () => ({latitude_deg: 0, longitude_deg: 0, bearing_deg: 0})
          }
      },
  
      data() {
          return {
              // Default Center at MDRS
              center: L.latLng(38.4225202, -110.7844653,),
              attribution: '&copy; <a href="http://osm.org/copyright">OpenStreetMap</a> contributors',
              online: true,
              onlineUrl: onlineUrl,
              offlineUrl: offlineUrl,
              onlineTileOptions: onlineTileOptions,
              offlineTileOptions: offlineTileOptions,
              roverMarker: null,
              droneMarker: null,
              waypointIcon: null,
              highlightedWaypointIcon: null,
              map: null,
              odomCount: 0,
              droneCount: 0,
              locationIcon: null,
              droneIcon: null,
              odomPath: [],
              dronePath: [],
              findRover: false,
              circle: null, //search radius
          }
      },
  
      created: function () {
          this.locationIcon = L.icon({
              iconUrl: '/location_marker_icon.png',
              iconSize: [64, 64],
              iconAnchor: [32, 32]
          })
          this.droneIcon = L.icon({
              iconUrl: '/drone_icon_1.png',
              iconSize: [64, 64],
              iconAnchor: [32, 32]
          })
          this.waypointIcon = L.icon({
              iconUrl: '/map_marker.png',
              iconSize: [64, 64],
              iconAnchor: [32, 64],
              popupAnchor: [0, -32]
          })
          this.droneWaypointIcon = L.icon({
              iconUrl: '/map_marker_drone.png',
              iconSize: [64, 64],
              iconAnchor: [32, 64],
              popupAnchor: [0, -32]
          })
          this.highlightedWaypointIcon = L.icon({
              iconUrl: '/map_marker_highlighted.png',
              iconSize: [64, 64],
              iconAnchor: [32, 64],
              popupAnchor: [0, -32]
          })
      },
  
      methods: {
        ...mapActions('websocket', ['sendMessage']),
          onMapReady: function () {
              // Pull objects from refs to be able to access data and change w functions
              this.$nextTick(() => {
                  this.map = this.$refs.map.leafletObject
                  this.roverMarker = this.$refs.rover.leafletObject
                  this.droneMarker = this.$refs.drone.leafletObject
              })
          },
          // Event listener for setting store values to get data to waypoint Editor
          getClickedLatLon: function (e: any) {
              this.setClickPoint({
                  lat: e.latlng.lat,
                  lon: e.latlng.lng
              })
          },
          getWaypointIcon: function (waypoint: any, index: number) {
              if (index === this.highlightedWaypoint) {
                  return this.highlightedWaypointIcon
              } else if (waypoint.drone) {
                  return this.droneWaypointIcon
              } else {
                  return this.waypointIcon
              }
          },
          ...mapMutations('erd', {
              setClickPoint: 'setClickPoint',
              setWaypointList: 'setWaypointList',
              setOdomFormat: 'setOdomFormat'
          })
      },
  
      computed: {
         ...mapState('websocket', ['message']),

          ...mapGetters('erd', {
              waypointList: 'waypointList',
              highlightedWaypoint: 'highlightedWaypoint',
              searchWaypoint: 'searchWaypoint'
          }),
  
          // Convert to latLng object for Leaflet to use
          odomLatLng: function () {
            if (this.odom && typeof this.odom === 'object' && this.odom.latitude_deg !== undefined && this.odom.longitude_deg !== undefined ) {
                return L.latLng(this.odom.latitude_deg, this.odom.longitude_deg);
            } else {
                console.warn('odom data not ready yet');
                return L.latLng(0,0);
            }
          },

          droneLatLng: function () {
            if (this.drone_odom && typeof this.drone_odom === 'object' && this.drone_odom.latitude_deg !== undefined && this.drone_odom.longitude_deg !== undefined ) {
                return L.latLng(this.drone_odom.latitude_deg, this.drone_odom.longitude_deg);
            } else {
                console.warn('drone odom data not ready yet');
                return L.latLng(0,0);
            }
          },
  
          polylinePath: function () {
              return [this.odomLatLng].concat(this.route.map((waypoint: any) => waypoint.latLng))
          },

          dronepolylinePath: function () {
              return [this.droneLatLng].concat(this.route.map((waypoint: any) => waypoint.latLng))
          },
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
                  this.roverMarker.setRotationAngle(angle)
  
                  this.roverMarker.setLatLng(latLng)
  
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
          },

          drone_odom: {
            handler: function(val) {
                const lat = val.latitude_deg
                const lng = val.longitude_deg
  
                const latLng = L.latLng(lat, lng)
                this.droneMarker.setLatLng(latLng)
                // Update the rover path
                this.droneCount++
                if (this.droneCount % DRAW_FREQUENCY === 0) {
                    if (this.dronePath.length > MAX_ODOM_COUNT) {
                        this.dronePath = [...this.dronePath.slice(1), latLng] //remove oldest element
                    }

                    this.dronePath = [...this.dronePath, latLng]
                    this.droneCount = 0
                }
            },
            deep: true
          },

          searchWaypoint(newIndex) {
            if(newIndex === -1){
                this.map.removeLayer(this.circle);
                this.circle = null;
                return;
            }
            const waypoint = this.waypointList[newIndex];
            if(!this.circle) this.circle = L.circle(waypoint.latLng, {radius: 20}).addTo(this.map);
            else this.circle.setLatLng(waypoint.latLng)
            this.circle.setStyle({fillColor: 'purple', stroke: false})
          }
      }
  }
  </script>
  
  <style scoped>
  .map {
      height: 100%;
      width: 100%;
  }
  
  .wrap {
      display: flex;
      align-items: center;
      height: 100%;
      width: 100%;
  }
  </style>
  
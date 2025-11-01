<template>
  <div class="wrapper d-flex m-0 p-2 justify-content-between gap-3 w-100 h-100">
    <div class="d-flex flex-column w-100 gap-2">
      <h3 class="m-0 p-0">Add Waypoint</h3>
      <div class="form-group d-flex gap-2 align-items-center">
        <label for="waypointname" class="form-label m-0 p-0">Name:</label>
        <input class="form-control" id="waypointname" v-model="name" />
      </div>


      <div class="d-flex gap-2">
        <div class="d-flex flex-column border border-2 rounded p-2">
          <div class="d-flex justify-content-between">
            <label class="form-label">Latitude:</label>
            <div class="col-auto">N</div>
          </div>
          <div class="col input-group">
            <input class="form-control" id="deg1" v-model.number="input.lat.d" />
            <span class="input-group-text font-monospace">º</span>
          </div>
        </div>
        <div class="d-flex flex-column border border-2 rounded p-2">
          <div class="d-flex justify-content-between">
            <label class="form-label">Longitude:</label>
            <div class="col-auto">W</div>
          </div>
          <div class="col input-group">
            <input class="form-control" id="deg2" v-model.number="input.lon.d" />
            <span class="input-group-text font-monospace">º</span>
          </div>
        </div>
      </div>
      
      <div class="d-flex flex-column gap-2">
        <button class="btn btn-success" @click="addWaypoint(input, false)">
          Add Waypoint
        </button>
        <button
          class="btn btn-success"
          @click="addWaypoint(formatted_odom, false)"
        >
          Drop Waypoint at Rover
        </button>
        <button
          v-if="droneWaypointButton"
          class="btn btn-info"
          @click="addWaypoint(input, true)"
        >
          Add Drone Position
        </button>
      </div>
    </div>

    <div class="d-flex flex-column w-100">
      <div class="d-flex mb-2 align-items-center justify-content-between">
        <h3 class="m-0 p-0">Current Course</h3>
        <button class="btn btn-danger" @click="clearWaypoint">Clear</button>
      </div>
      <div class="waypoint-wrapper overflow-y-scroll d-flex flex-column gap-2">
        <WaypointItem
          v-for="(waypoint, i) in storedWaypoints"
          :key="i"
          :waypoint="waypoint"
          :index="i"
          @delete="deleteItem($event)"
          @find="findWaypoint($event)"
          @search="searchForWaypoint($event)"
        />
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch, onMounted } from 'vue'
import WaypointItem from './BasicWaypointItem.vue'
import { useErdStore } from '@/stores/erd'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import L from 'leaflet'
import { waypointsAPI } from '@/utils/api'
import type { StoreWaypoint, APIBasicWaypoint } from '@/types/waypoints'
import type { NavMessage } from '@/types/coordinates'

defineProps({
  droneWaypointButton: {
    type: Boolean,
    required: false,
  },
})

const erdStore = useErdStore()
const { highlightedWaypoint, searchWaypoint, clickPoint } = storeToRefs(erdStore)
const { setWaypointList, setHighlightedWaypoint, setSearchWaypoint } = erdStore

const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const rover_latitude_deg = ref(0)
const rover_longitude_deg = ref(0)

const name = ref('Waypoint')
const input = ref({
  lat: { d: 0 },
  lon: { d: 0 },
})
const storedWaypoints = ref<StoreWaypoint[]>([])

const formatted_odom = computed(() => {
  return {
    lat: { d: rover_latitude_deg.value },
    lon: { d: rover_longitude_deg.value },
  }
})

const navMessage = computed(() => messages.value['nav'])

watch(navMessage, (msg) => {
  if (!msg) return
  const navMsg = msg as NavMessage

  if (navMsg.type === 'gps_fix') {
    rover_latitude_deg.value = navMsg.latitude
    rover_longitude_deg.value = navMsg.longitude
  }
})

const saveWaypoints = async (waypoints: StoreWaypoint[]) => {
  try {
    const apiWaypoints: APIBasicWaypoint[] = waypoints.map(wp => ({
      name: wp.name,
      lat: wp.latLng.lat,
      lon: wp.latLng.lng,
      drone: wp.drone,
    }));
    await waypointsAPI.saveBasic(apiWaypoints)
  } catch (error) {
    console.error('Failed to save waypoints:', error)
  }
}

const loadWaypoints = async () => {
  try {
    const data = await waypointsAPI.getBasic()
    if (data.status === 'success' && data.waypoints) {
      storedWaypoints.value = data.waypoints.map((wp: APIBasicWaypoint) => ({
        name: wp.name,
        latLng: L.latLng(wp.lat, wp.lon),
        drone: wp.drone,
      }));
      setWaypointList(storedWaypoints.value)
    }
  } catch (error) {
    console.error('Failed to load waypoints:', error)
  }
}

const deleteItem = (payload: { index: number }) => {
  if (highlightedWaypoint.value == payload.index) {
    setHighlightedWaypoint(-1)
  }
  if (searchWaypoint.value == payload.index) {
    setSearchWaypoint(-1)
  }
  storedWaypoints.value.splice(payload.index, 1)
}

const addWaypoint = (
  coord: {
    lat: { d: number }
    lon: { d: number }
  },
  isDrone: boolean,
) => {
  storedWaypoints.value.push({
    name: name.value,
    latLng: L.latLng(coord.lat.d, coord.lon.d),
    drone: isDrone,
  })
}

const findWaypoint = (payload: { index: number }) => {
  if (payload.index === highlightedWaypoint.value) {
    setHighlightedWaypoint(-1)
  } else {
    setHighlightedWaypoint(payload.index)
  }
}

const searchForWaypoint = (payload: { index: number }) => {
  if (payload.index === searchWaypoint.value) {
    setSearchWaypoint(-1)
  } else {
    setSearchWaypoint(payload.index)
  }
}

const clearWaypoint = () => {
  storedWaypoints.value = []
}

watch(storedWaypoints, (newList) => {
  setWaypointList(newList)
  saveWaypoints(newList)
}, { deep: true })

watch(clickPoint, (newClickPoint) => {
  input.value.lat.d = newClickPoint.lat
  input.value.lon.d = newClickPoint.lon
})

onMounted(() => {
  setHighlightedWaypoint(-1)
  setSearchWaypoint(-1)
  setWaypointList([])

  setTimeout(() => {
    loadWaypoints()
  }, 250)
})
</script>

<style scoped>
.waypoint-wrapper {
  flex: 1;
  overflow-y: auto;
  background-color: #dddddd;
  padding: 8px;
  border-radius: 8px;
}

.waypoint-col {
  min-width: 300px;
}
</style>

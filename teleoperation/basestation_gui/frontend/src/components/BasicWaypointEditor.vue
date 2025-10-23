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
import { ref, computed, watch, onMounted, defineProps } from 'vue'
import WaypointItem from './BasicWaypointItem.vue'
import { useErdStore } from '@/stores/erd'
import { storeToRefs } from 'pinia'
import L from 'leaflet'
import { waypointsAPI } from '@/utils/api'

const props = defineProps({
  odom: {
    type: Object,
    default: () => ({ latitude_deg: 0, longitude_deg: 0, bearing_deg: 0 }),
  },
  droneWaypointButton: {
    type: Boolean,
    required: false,
  },
})

const erdStore = useErdStore()
const { highlightedWaypoint, searchWaypoint, clickPoint } = storeToRefs(erdStore)
const { setWaypointList, setHighlightedWaypoint, setSearchWaypoint } = erdStore

const name = ref('Waypoint')
const input = ref({
  lat: { d: 0 },
  lon: { d: 0 },
})
const storedWaypoints = ref<any[]>([])

const formatted_odom = computed(() => {
  return {
    lat: { d: props.odom.latitude_deg },
    lon: { d: props.odom.longitude_deg },
  }
})

const saveWaypoints = async (waypoints: any[]) => {
  try {
    await waypointsAPI.saveBasic(waypoints)
  } catch (error) {
    console.error('Failed to save waypoints:', error)
  }
}

const loadWaypoints = async () => {
  try {
    const data = await waypointsAPI.getBasic()
    if (data.status === 'success' && data.waypoints) {
      storedWaypoints.value = data.waypoints
      const waypoints = data.waypoints.map(
        (waypoint: { lat: number; lon: number; name: string }) => ({
          latLng: L.latLng(waypoint.lat, waypoint.lon),
          name: waypoint.name
        })
      )
      setWaypointList(waypoints)
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
    lat: (coord.lat.d).toFixed(5),
    lon: (coord.lon.d).toFixed(5),
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
  const waypoints = newList.map(
    (waypoint: {
      lat: number
      lon: number
      name: string
      drone: boolean
    }) => {
      return {
        latLng: L.latLng(waypoint.lat, waypoint.lon),
        name: waypoint.name,
        drone: waypoint.drone,
      }
    },
  )
  setWaypointList(waypoints)
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

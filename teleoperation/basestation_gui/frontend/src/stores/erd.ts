import { defineStore } from 'pinia'
import { ref, computed } from 'vue'
import L from 'leaflet'
import type { BasicWaypointRecord, MapWaypoint } from '@/types/waypoints'
import { waypointsAPI } from '@/utils/api'

export const useErdStore = defineStore('erd', () => {
  const waypoints = ref<BasicWaypointRecord[]>([])
  const highlightedWaypoint = ref(-1)
  const searchWaypoint = ref(-1)
  const odomFormat = ref('DM')
  const clickPoint = ref({ lat: 0, lon: 0 })

  const waypointListForMap = computed<MapWaypoint[]>(() =>
    waypoints.value.map(wp => ({
      latLng: L.latLng(wp.lat, wp.lon),
      name: wp.name,
      drone: wp.drone
    }))
  )

  async function fetchAll() {
    try {
      const data = await waypointsAPI.getBasic()
      if (data.status === 'success' && data.waypoints) {
        waypoints.value = data.waypoints
      }
    } catch (error) {
      console.error('Failed to load waypoints:', error)
    }
  }

  async function addWaypoint(wp: Omit<BasicWaypointRecord, 'db_id'>) {
    const resp = await waypointsAPI.createBasic(wp)
    if (resp.status === 'success' && resp.waypoint) {
      waypoints.value = [...waypoints.value, resp.waypoint]
    }
    return resp
  }

  async function deleteWaypoint(index: number) {
    const wp = waypoints.value[index]
    if (!wp) return

    const next = [...waypoints.value]
    next.splice(index, 1)
    waypoints.value = next

    if (highlightedWaypoint.value === index) highlightedWaypoint.value = -1
    if (searchWaypoint.value === index) searchWaypoint.value = -1

    if (wp.db_id != null) {
      try {
        await waypointsAPI.deleteBasicWaypoint(wp.db_id)
      } catch (error) {
        console.error('Failed to delete waypoint:', error)
        waypoints.value = [...waypoints.value.slice(0, index), wp, ...waypoints.value.slice(index)]
      }
    }
  }

  async function clearAll() {
    await waypointsAPI.deleteAll()
    waypoints.value = []
    highlightedWaypoint.value = -1
    searchWaypoint.value = -1
  }

  function setHighlighted(index: number) {
    highlightedWaypoint.value = highlightedWaypoint.value === index ? -1 : index
  }

  function setSearch(index: number) {
    searchWaypoint.value = searchWaypoint.value === index ? -1 : index
  }

  return {
    waypoints,
    highlightedWaypoint,
    searchWaypoint,
    odomFormat,
    clickPoint,
    waypointListForMap,
    fetchAll,
    addWaypoint,
    deleteWaypoint,
    clearAll,
    setHighlighted,
    setSearch,
  }
})

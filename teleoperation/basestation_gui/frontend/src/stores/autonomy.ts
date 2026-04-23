import { defineStore } from 'pinia'
import { ref, computed } from 'vue'
import L from 'leaflet'
import type { AutonWaypoint, MapWaypoint, MapRouteWaypoint } from '@/types/waypoints'
import { waypointsAPI } from '@/utils/api'

export const useAutonomyStore = defineStore('autonomy', () => {
  const waypoints = ref<AutonWaypoint[]>([])
  const route = ref<AutonWaypoint[]>([])
  const allCostmapToggle = ref(true)

  const highlightedWaypoint = ref(-1)
  const autonEnabled = ref(false)
  const teleopEnabled = ref(false)
  const purePursuitEnabled = ref(false)
  const pathRelaxationEnabled = ref(false)
  const pathInterpolationEnabled = ref(false)
  const odomFormat = ref('DM')
  const clickPoint = ref({ lat: 0, lon: 0 })

  const waypointListForMap = computed<MapWaypoint[]>(() =>
    waypoints.value.map(wp => ({
      latLng: L.latLng(wp.lat, wp.lon),
      name: wp.name
    }))
  )

  const routeForMap = computed<MapRouteWaypoint[]>(() =>
    route.value.map(wp => ({
      latLng: L.latLng(wp.lat, wp.lon),
      name: wp.name,
      tag_id: wp.tag_id,
      type: wp.type,
      enable_costmap: wp.enable_costmap
    }))
  )

  function isInRoute(wp: AutonWaypoint): boolean {
    return route.value.some(
      r => r.name === wp.name && r.tag_id === wp.tag_id && r.type === wp.type
    )
  }

  async function fetchAll() {
    try {
      const autonData = await waypointsAPI.getAuton()
      if (autonData.status === 'success') {
        waypoints.value = autonData.waypoints || []
      }

      const courseData = await waypointsAPI.getCurrentAutonCourse()
      if (courseData.status === 'success') {
        route.value = courseData.course || []
      }
    } catch (error) {
      console.error('Failed to load waypoints:', error)
    }
  }

  async function createWaypoint(payload: Omit<AutonWaypoint, 'db_id' | 'deletable'>) {
    const resp = await waypointsAPI.createAuton(payload)
    if (resp.status === 'success' && resp.waypoint) {
      waypoints.value = [...waypoints.value, resp.waypoint]
    }
    return resp
  }

  async function updateWaypoint(dbId: number, fields: Partial<AutonWaypoint>) {
    const resp = await waypointsAPI.updateAuton(dbId, fields)
    if (resp.status === 'success' && resp.waypoint) {
      waypoints.value = waypoints.value.map(wp =>
        wp.db_id === dbId ? resp.waypoint! : wp
      )
    }
    return resp
  }

  async function deleteWaypoint(index: number) {
    const wp = waypoints.value[index]
    if (!wp) return

    const next = [...waypoints.value]
    next.splice(index, 1)
    waypoints.value = next

    if (wp.db_id != null && wp.deletable) {
      try {
        await waypointsAPI.deleteAutonWaypoint(wp)
      } catch (error) {
        console.error('Failed to delete waypoint:', error)
        waypoints.value = [...waypoints.value.slice(0, index), wp, ...waypoints.value.slice(index)]
      }
    }
  }

  async function addToRoute(waypoint: AutonWaypoint) {
    const newPoint = { ...waypoint, enable_costmap: allCostmapToggle.value }
    route.value = [...route.value, newPoint]
    await saveRoute()
  }

  async function removeFromRoute(waypoint: AutonWaypoint) {
    const index = route.value.indexOf(waypoint)
    if (index > -1) {
      const next = [...route.value]
      next.splice(index, 1)
      route.value = next
    }
    await saveRoute()
  }

  async function saveRoute() {
    try {
      await waypointsAPI.saveCurrentAutonCourse(route.value)
    } catch (error) {
      console.error('Failed to save current auton course:', error)
    }
  }

  async function toggleRouteCostmap(payload: { waypoint: AutonWaypoint; enable_costmap: boolean }) {
    route.value = route.value.map(wp =>
      wp === payload.waypoint ? { ...wp, enable_costmap: payload.enable_costmap } : wp
    )
    await saveRoute()
  }

  async function toggleAllCostmaps() {
    allCostmapToggle.value = !allCostmapToggle.value
    route.value = route.value.map(wp => ({ ...wp, enable_costmap: allCostmapToggle.value }))
    await saveRoute()
  }

  async function resetUserWaypoints() {
    await waypointsAPI.clearAuton()
    await fetchAll()
  }

  return {
    waypoints,
    route,
    allCostmapToggle,
    highlightedWaypoint,
    autonEnabled,
    teleopEnabled,
    purePursuitEnabled,
    pathRelaxationEnabled,
    pathInterpolationEnabled,
    odomFormat,
    clickPoint,
    waypointListForMap,
    routeForMap,
    isInRoute,
    fetchAll,
    createWaypoint,
    updateWaypoint,
    deleteWaypoint,
    addToRoute,
    removeFromRoute,
    saveRoute,
    toggleRouteCostmap,
    toggleAllCostmaps,
    resetUserWaypoints,
  }
})

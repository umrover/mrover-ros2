  import { defineStore } from 'pinia'
import { ref } from 'vue'
import type { StoreWaypoint, AutonRouteWaypoint } from '@/types/waypoints'

export const useAutonomyStore = defineStore('autonomy', () => {
  // State
  const route = ref<AutonRouteWaypoint[]>([])
  const waypointList = ref<StoreWaypoint[]>([])
  const highlightedWaypoint = ref(-1)
  const autonEnabled = ref(false)
  const teleopEnabled = ref(false)
  const purePursuitEnabled = ref(false)
  const pathRelaxationEnabled = ref(false)
  const pathInterpolationEnabled = ref(false)
  const odomFormat = ref('DM')
  const clickPoint = ref({ lat: 0, lon: 0 })

  // Actions
  function setRoute(newRoute: AutonRouteWaypoint[]) {
    route.value = newRoute
  }

  function setAutonMode(newAutonEnabled: boolean) {
    autonEnabled.value = newAutonEnabled
  }

  function setTeleopMode(newTeleopEnabled: boolean) {
    teleopEnabled.value = newTeleopEnabled
  }

  function setPurePursuit(enabled: boolean) {
    purePursuitEnabled.value = enabled
  }

  function setPathRelaxation(enabled: boolean) {
    pathRelaxationEnabled.value = enabled
  }

  function setPathInterpolation(enabled: boolean) {
    pathInterpolationEnabled.value = enabled
  }

  function setWaypointList(newList: StoreWaypoint[]) {
    waypointList.value = newList
  }

  function setHighlightedWaypoint(newWaypoint: number) {
    highlightedWaypoint.value = newWaypoint
  }

  function setOdomFormat(newOdomFormat: string) {
    odomFormat.value = newOdomFormat
  }

  function setClickPoint(newClickPoint: { lat: number; lon: number }) {
    clickPoint.value = newClickPoint
  }

  return {
    // State
    route,
    waypointList,
    highlightedWaypoint,
    autonEnabled,
    teleopEnabled,
    purePursuitEnabled,
    pathRelaxationEnabled,
    pathInterpolationEnabled,
    odomFormat,
    clickPoint,
    // Actions
    setRoute,
    setAutonMode,
    setTeleopMode,
    setPurePursuit,
    setPathRelaxation,
    setPathInterpolation,
    setWaypointList,
    setHighlightedWaypoint,
    setOdomFormat,
    setClickPoint,
  }
})
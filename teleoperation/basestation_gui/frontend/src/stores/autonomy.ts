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

  return {
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
  }
})
  import { defineStore } from 'pinia'
import { ref, computed } from 'vue'
import type { StoreWaypoint } from '@/types/waypoints'

export const useAutonomyStore = defineStore('autonomy', () => {
  // State
  const route = ref<StoreWaypoint[]>([])
  const waypointList = ref<StoreWaypoint[]>([])
  const highlightedWaypoint = ref(-1)
  const autonEnabled = ref(false)
  const teleopEnabled = ref(false)
  const odomFormat = ref('DM')
  const clickPoint = ref({ lat: 0, lon: 0 })

  // Getters
  const getRoute = computed(() => route.value)
  const getWaypointList = computed(() => waypointList.value)
  const getHighlightedWaypoint = computed(() => highlightedWaypoint.value)
  const isAutonEnabled = computed(() => autonEnabled.value)
  const isTeleopEnabled = computed(() => teleopEnabled.value)
  const getOdomFormat = computed(() => odomFormat.value)
  const getClickPoint = computed(() => clickPoint.value)

  // Actions
  function setRoute(newRoute: StoreWaypoint[]) {
    route.value = newRoute
  }

  function setAutonMode(newAutonEnabled: boolean) {
    autonEnabled.value = newAutonEnabled
  }

  function setTeleopMode(newTeleopEnabled: boolean) {
    teleopEnabled.value = newTeleopEnabled
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
    odomFormat,
    clickPoint,
    // Getters
    getRoute,
    getWaypointList,
    getHighlightedWaypoint,
    isAutonEnabled,
    isTeleopEnabled,
    getOdomFormat,
    getClickPoint,
    // Actions
    setRoute,
    setAutonMode,
    setTeleopMode,
    setWaypointList,
    setHighlightedWaypoint,
    setOdomFormat,
    setClickPoint,
  }
})
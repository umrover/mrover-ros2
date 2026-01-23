import { defineStore } from 'pinia'
import { ref, computed } from 'vue'
import type { StoreWaypoint } from '@/types/waypoints'

export const useErdStore = defineStore('erd', () => {
  // State
  const waypointList = ref<StoreWaypoint[]>([])
  const highlightedWaypoint = ref(-1)
  const searchWaypoint = ref(-1)
  const odomFormat = ref('DM')
  const clickPoint = ref({ lat: 0, lon: 0 })

  // Getters
  const getWaypointList = computed(() => waypointList.value)
  const getHighlightedWaypoint = computed(() => highlightedWaypoint.value)
  const getSearchWaypoint = computed(() => searchWaypoint.value)
  const getOdomFormat = computed(() => odomFormat.value)
  const getClickPoint = computed(() => clickPoint.value)

  // Actions
  function setWaypointList(newList: StoreWaypoint[]) {
    waypointList.value = newList
  }

  function setHighlightedWaypoint(newWaypoint: number) {
    highlightedWaypoint.value = newWaypoint
  }

  function setSearchWaypoint(newWaypoint: number) {
    searchWaypoint.value = newWaypoint
  }

  function setOdomFormat(newOdomFormat: string) {
    odomFormat.value = newOdomFormat
  }

  function setClickPoint(newClickPoint: { lat: number; lon: number }) {
    clickPoint.value = newClickPoint
  }

  return {
    // State
    waypointList,
    highlightedWaypoint,
    searchWaypoint,
    odomFormat,
    clickPoint,
    // Getters
    getWaypointList,
    getHighlightedWaypoint,
    getSearchWaypoint,
    getOdomFormat,
    getClickPoint,
    // Actions
    setWaypointList,
    setHighlightedWaypoint,
    setSearchWaypoint,
    setOdomFormat,
    setClickPoint,
  }
})
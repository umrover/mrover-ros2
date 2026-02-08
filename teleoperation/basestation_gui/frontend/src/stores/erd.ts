import { defineStore } from 'pinia'
import { ref } from 'vue'
import type { StoreWaypoint } from '@/types/waypoints'

export const useErdStore = defineStore('erd', () => {
  // State
  const waypointList = ref<StoreWaypoint[]>([])
  const highlightedWaypoint = ref(-1)
  const searchWaypoint = ref(-1)
  const odomFormat = ref('DM')
  const clickPoint = ref({ lat: 0, lon: 0 })

  return {
    waypointList,
    highlightedWaypoint,
    searchWaypoint,
    odomFormat,
    clickPoint,
  }
})
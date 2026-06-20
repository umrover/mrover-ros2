import { defineStore } from 'pinia'
import { ref, computed } from 'vue'
import type { ScienceWaypointRecord } from '@/types/waypoints'
import { scienceWaypointsAPI } from '@/utils/api'
import { currentTimestamp } from '@/utils/formatNumber'

export const useScienceWaypointStore = defineStore('scienceWaypoints', () => {
  const waypoints = ref<ScienceWaypointRecord[]>([])
  const highlightedWaypoint = ref(-1)
  const searchWaypoint = ref(-1)
  const clickPoint = ref({ lat: 0, lon: 0 })

  const nextName = computed(() => `Waypoint ${waypoints.value.length + 1}`)

  async function fetchAll() {
    try {
      const data = await scienceWaypointsAPI.getAll()
      if (data.status === 'success' && data.waypoints) {
        waypoints.value = data.waypoints
      }
    } catch (error) {
      console.error('Failed to load science waypoints:', error)
    }
  }

  async function addWaypoint(wp: { name: string; lat: number; lon: number; altitude: number }) {
    const resp = await scienceWaypointsAPI.create(wp)
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

    if (wp.id != null) {
      try {
        await scienceWaypointsAPI.delete(wp.id)
      } catch (error) {
        console.error('Failed to delete science waypoint:', error)
        waypoints.value = [...waypoints.value.slice(0, index), wp, ...waypoints.value.slice(index)]
      }
    }
  }

  async function clearAll() {
    await scienceWaypointsAPI.clear()
    waypoints.value = []
    highlightedWaypoint.value = -1
    searchWaypoint.value = -1
  }

  async function resetTable() {
    await scienceWaypointsAPI.reset()
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

  function exportToText() {
    if (waypoints.value.length === 0) return

    let textContent = `Generated: ${new Date().toLocaleString()}\n\n`
    waypoints.value.forEach((wp, index) => {
      textContent += `[${index + 1}] ${wp.name.toUpperCase()}\n`
      textContent += `    Latitude:  ${wp.lat.toFixed(8)}\n`
      textContent += `    Longitude: ${wp.lon.toFixed(8)}\n`
      textContent += `    Altitude:  ${wp.altitude.toFixed(4)} m\n\n`
    })

    textContent += '0.01 m'
    const blob = new Blob([textContent.trim()], { type: 'text/plain;charset=utf-8;' })
    const link = document.createElement('a')
    const url = URL.createObjectURL(blob)
    link.setAttribute('href', url)
    link.setAttribute('download', `science_course_${currentTimestamp()}.txt`)
    link.style.visibility = 'hidden'
    document.body.appendChild(link)
    link.click()
    document.body.removeChild(link)
    URL.revokeObjectURL(url)
  }

  return {
    waypoints,
    highlightedWaypoint,
    searchWaypoint,
    clickPoint,
    nextName,
    fetchAll,
    addWaypoint,
    deleteWaypoint,
    clearAll,
    resetTable,
    setHighlighted,
    setSearch,
    exportToText,
  }
})

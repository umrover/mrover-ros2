import { defineStore } from 'pinia'
import { ref, computed } from 'vue'
import L from 'leaflet'
import type { AutonWaypoint, MapWaypoint, MapRouteWaypoint } from '@/types/waypoints'
<<<<<<< HEAD
import { waypointsAPI } from '@/utils/api'

export const useAutonomyStore = defineStore('autonomy', () => {
  const waypoints = ref<AutonWaypoint[]>([])
  const route = ref<AutonWaypoint[]>([])
  const allCostmapToggle = ref(true)
=======
import { waypointsAPI, autonAPI } from '@/utils/api'

function isInList(list: AutonWaypoint[], wp: AutonWaypoint): boolean {
  return wp.db_id != null && list.some(item => item.db_id === wp.db_id)
}

export const useAutonomyStore = defineStore('autonomy', () => {
  const store = ref<AutonWaypoint[]>([])
  const execution = ref<AutonWaypoint[]>([])
  const allCostmapToggle = ref(true)
  const navState = ref('OffState')

  const isNavigating = computed(() =>
    navState.value !== 'OffState' && navState.value !== 'DoneState'
  )
>>>>>>> origin/main

  const highlightedWaypoint = ref(-1)
  const autonEnabled = ref(false)
  const teleopEnabled = ref(false)
<<<<<<< HEAD
  const purePursuitEnabled = ref(false)
=======
  const purePursuitEnabled = ref(true)
>>>>>>> origin/main
  const pathRelaxationEnabled = ref(false)
  const pathInterpolationEnabled = ref(false)
  const odomFormat = ref('DM')
  const clickPoint = ref({ lat: 0, lon: 0 })

<<<<<<< HEAD
  const waypointListForMap = computed<MapWaypoint[]>(() =>
    waypoints.value.map(wp => ({
=======
  const storeForMap = computed<MapWaypoint[]>(() =>
    store.value.map(wp => ({
>>>>>>> origin/main
      latLng: L.latLng(wp.lat, wp.lon),
      name: wp.name
    }))
  )

<<<<<<< HEAD
  const routeForMap = computed<MapRouteWaypoint[]>(() =>
    route.value.map(wp => ({
=======
  const executionForMap = computed<MapRouteWaypoint[]>(() =>
    execution.value.map(wp => ({
>>>>>>> origin/main
      latLng: L.latLng(wp.lat, wp.lon),
      name: wp.name,
      tag_id: wp.tag_id,
      type: wp.type,
<<<<<<< HEAD
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
=======
      enable_costmap: wp.enable_costmap,
    }))
  )

  // --- Fetch ---

  async function fetchStore() {
    const resp = await waypointsAPI.getStore()
    if (resp.status === 'success') {
      store.value = resp.waypoints || []
    }
  }

  async function fetchExecution() {
    const resp = await waypointsAPI.getExecution()
    if (resp.status === 'success') {
      execution.value = resp.course || []
    }
  }

  async function fetchAll() {
    try {
      await Promise.all([fetchStore(), fetchExecution()])
    } catch (error) {
      console.error('Failed to load waypoints:', error)
    }
  }

  // --- Store: add, update, remove, reorder ---

  async function addToStore(payload: Omit<AutonWaypoint, 'db_id' | 'deletable'>) {
    const resp = await waypointsAPI.addToStore(payload)
    if (resp.status === 'success' && resp.waypoint) {
      store.value = [...store.value, resp.waypoint]
>>>>>>> origin/main
    }
    return resp
  }

<<<<<<< HEAD
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
=======
  async function updateStore(dbId: number, fields: Partial<AutonWaypoint>) {
    const resp = await waypointsAPI.updateStore(dbId, fields)
    if (resp.status === 'success' && resp.waypoint) {
      store.value = store.value.map(wp =>
        wp.db_id === dbId ? resp.waypoint! : wp
      )
    }
    return resp
  }

  async function removeFromStore(index: number) {
    const wp = store.value[index]
    if (!wp || wp.db_id == null) return

    try {
      await waypointsAPI.removeFromStore(wp)
      const next = [...store.value]
      next.splice(index, 1)
      store.value = next
    } catch (error) {
      console.error('Failed to delete waypoint:', error)
    }
  }

  async function removeMultipleFromStore(indices: number[]) {
    const sorted = [...indices].sort((a, b) => b - a)
    const toDelete = sorted
      .map(i => store.value[i])
      .filter((wp): wp is AutonWaypoint => wp != null && wp.db_id != null)

    try {
      await Promise.all(toDelete.map(wp => waypointsAPI.removeFromStore(wp)))
      const idsToRemove = new Set(toDelete.map(wp => wp.db_id))
      store.value = store.value.filter(wp => !idsToRemove.has(wp.db_id))
    } catch (error) {
      console.error('Failed to delete waypoints:', error)
    }
  }

  function reorderStore(fromIndex: number, toIndex: number) {
    const next = [...store.value]
    const [moved] = next.splice(fromIndex, 1)
    next.splice(toIndex, 0, moved)
    store.value = next
  }

  // --- Execution: add, remove, save ---

  async function addToExecution(waypoint: AutonWaypoint) {
    if (isInList(execution.value, waypoint)) return
    execution.value = [...execution.value, { ...waypoint, enable_costmap: waypoint.enable_costmap ?? allCostmapToggle.value }]
    await saveExecution()
  }

  async function addManyToExecution(waypoints: AutonWaypoint[]) {
    const toAdd = waypoints.filter(wp => !isInList(execution.value, wp))
    if (toAdd.length === 0) return
    execution.value = [...execution.value, ...toAdd.map(wp => ({ ...wp, enable_costmap: wp.enable_costmap ?? allCostmapToggle.value }))]
    await saveExecution()
  }

  async function removeFromExecution(waypoint: AutonWaypoint) {
    const index = execution.value.findIndex(wp => wp.db_id === waypoint.db_id)
    if (index === -1) return
    const next = [...execution.value]
    next.splice(index, 1)
    execution.value = next
    await saveExecution()
  }

  async function saveExecution() {
    try {
      await waypointsAPI.saveExecution(execution.value)
    } catch (error) {
      console.error('Failed to save execution:', error)
    }
  }

  function reorderExecution(fromIndex: number, toIndex: number) {
    const next = [...execution.value]
    const [moved] = next.splice(fromIndex, 1)
    next.splice(toIndex, 0, moved)
    execution.value = next
    saveExecution()
  }

  // --- Composite operations ---

  async function clearExecution() {
    try {
      await autonAPI.enable(false, [])
      autonEnabled.value = false
    } catch (error) {
      console.error('Failed to disable autonomy:', error)
    }
    execution.value = []
    await saveExecution()
  }

  function setAllCostmaps(value: boolean) {
    allCostmapToggle.value = value
    execution.value = execution.value.map(wp => ({ ...wp, enable_costmap: value }))
    saveExecution()
  }

  function toggleAllCostmaps() {
    setAllCostmaps(!allCostmapToggle.value)
  }

  function toggleExecutionCostmap(index: number) {
    const next = [...execution.value]
    const wp = next[index]
    if (!wp) return
    next[index] = { ...wp, enable_costmap: !wp.enable_costmap }
    execution.value = next
    saveExecution()
  }

  function setNavState(state: string) {
    navState.value = state
  }

  async function resetAll() {
    await waypointsAPI.resetStore()
>>>>>>> origin/main
    await fetchAll()
  }

  return {
<<<<<<< HEAD
    waypoints,
    route,
    allCostmapToggle,
=======
    store,
    execution,
    allCostmapToggle,
    navState,
    isNavigating,
>>>>>>> origin/main
    highlightedWaypoint,
    autonEnabled,
    teleopEnabled,
    purePursuitEnabled,
    pathRelaxationEnabled,
    pathInterpolationEnabled,
    odomFormat,
    clickPoint,
<<<<<<< HEAD
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
=======
    storeForMap,
    executionForMap,
    fetchStore,
    fetchExecution,
    fetchAll,
    addToStore,
    updateStore,
    removeFromStore,
    removeMultipleFromStore,
    reorderStore,
    addToExecution,
    addManyToExecution,
    removeFromExecution,
    saveExecution,
    reorderExecution,
    clearExecution,
    setAllCostmaps,
    toggleAllCostmaps,
    toggleExecutionCostmap,
    setNavState,
    resetAll,
>>>>>>> origin/main
  }
})

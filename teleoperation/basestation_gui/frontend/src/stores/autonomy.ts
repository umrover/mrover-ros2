import { defineStore } from 'pinia'
import { ref, computed } from 'vue'
import L from 'leaflet'
import type { AutonWaypoint, MapWaypoint, MapRouteWaypoint } from '@/types/waypoints'
import { waypointsAPI, autonAPI } from '@/utils/api'

function isInList(list: AutonWaypoint[], wp: AutonWaypoint): boolean {
  return wp.db_id != null && list.some(item => item.db_id === wp.db_id)
}

export const useAutonomyStore = defineStore('autonomy', () => {
  const store = ref<AutonWaypoint[]>([])
  const staging = ref<AutonWaypoint[]>([])
  const execution = ref<AutonWaypoint[]>([])
  const allCostmapToggle = ref(true)
  const navState = ref('OffState')

  const isNavigating = computed(() =>
    navState.value !== 'OffState' && navState.value !== 'DoneState'
  )

  const highlightedWaypoint = ref(-1)
  const autonEnabled = ref(false)
  const teleopEnabled = ref(false)
  const purePursuitEnabled = ref(false)
  const pathRelaxationEnabled = ref(false)
  const pathInterpolationEnabled = ref(false)
  const stereoDetectorEnabled = ref(false)
  const imageDetectorEnabled = ref(false)
  const odomFormat = ref('DM')
  const clickPoint = ref({ lat: 0, lon: 0 })

  const storeForMap = computed<MapWaypoint[]>(() =>
    store.value.map(wp => ({
      latLng: L.latLng(wp.lat, wp.lon),
      name: wp.name
    }))
  )

  const stagingForMap = computed<MapRouteWaypoint[]>(() =>
    staging.value.map(wp => ({
      latLng: L.latLng(wp.lat, wp.lon),
      name: wp.name,
      tag_id: wp.tag_id,
      type: wp.type,
    }))
  )

  const executionForMap = computed<MapRouteWaypoint[]>(() =>
    execution.value.map(wp => ({
      latLng: L.latLng(wp.lat, wp.lon),
      name: wp.name,
      tag_id: wp.tag_id,
      type: wp.type,
    }))
  )

  // --- Fetch ---

  async function fetchStore() {
    const resp = await waypointsAPI.getStore()
    if (resp.status === 'success') {
      store.value = resp.waypoints || []
    }
  }

  async function fetchStaging() {
    const resp = await waypointsAPI.getStaging()
    if (resp.status === 'success') {
      staging.value = resp.course || []
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
      await Promise.all([fetchStore(), fetchStaging(), fetchExecution()])
    } catch (error) {
      console.error('Failed to load waypoints:', error)
    }
  }

  // --- Store: add, update, remove ---

  async function addToStore(payload: Omit<AutonWaypoint, 'db_id' | 'deletable'>) {
    const resp = await waypointsAPI.addToStore(payload)
    if (resp.status === 'success' && resp.waypoint) {
      store.value = [...store.value, resp.waypoint]
    }
    return resp
  }

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

  // --- Staging: add, remove, save ---

  async function addToStaging(waypoint: AutonWaypoint) {
    if (isInList(staging.value, waypoint) || isInList(execution.value, waypoint)) return
    staging.value = [...staging.value, { ...waypoint }]
    await saveStaging()
  }

  async function removeFromStaging(waypoint: AutonWaypoint) {
    const index = staging.value.findIndex(wp => wp.db_id === waypoint.db_id)
    if (index === -1) return
    const next = [...staging.value]
    next.splice(index, 1)
    staging.value = next
    await saveStaging()
  }

  async function clearStaging() {
    staging.value = []
    await saveStaging()
  }

  async function stageAllToExecution() {
    const toMove = staging.value.filter(wp => !isInList(execution.value, wp))
    if (toMove.length === 0) return
    execution.value = [...execution.value, ...toMove]
    staging.value = []
    await Promise.all([saveStaging(), saveExecution()])
  }

  async function saveStaging() {
    try {
      await waypointsAPI.saveStaging(staging.value)
    } catch (error) {
      console.error('Failed to save staging:', error)
    }
  }

  // --- Execution: add, remove, save ---

  async function addToExecution(waypoint: AutonWaypoint) {
    if (isInList(execution.value, waypoint) || isInList(staging.value, waypoint)) return
    execution.value = [...execution.value, { ...waypoint }]
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

  async function stageToExecution(waypoint: AutonWaypoint) {
    const stagingIndex = staging.value.findIndex(wp => wp.db_id === waypoint.db_id)
    if (stagingIndex === -1) return

    const nextStaging = [...staging.value]
    nextStaging.splice(stagingIndex, 1)
    staging.value = nextStaging
    execution.value = [...execution.value, { ...waypoint }]
    await Promise.all([saveStaging(), saveExecution()])
  }

  async function saveExecution() {
    try {
      await waypointsAPI.saveExecution(execution.value)
    } catch (error) {
      console.error('Failed to save execution:', error)
    }
  }

  // --- Composite operations ---

  async function stageNext() {
    if (staging.value.length === 0) return

    const [next, ...remaining] = staging.value
    execution.value = [...execution.value, { ...next }]
    staging.value = remaining

    await Promise.all([saveStaging(), saveExecution()])
  }

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

  async function unstageOne(waypoint: AutonWaypoint) {
    const index = execution.value.findIndex(wp => wp.db_id === waypoint.db_id)
    if (index === -1) return

    const nextExecution = [...execution.value]
    nextExecution.splice(index, 1)
    execution.value = nextExecution
    staging.value = [waypoint, ...staging.value]
    await Promise.all([saveStaging(), saveExecution()])
  }

  async function unstageExecution() {
    staging.value = [...execution.value, ...staging.value]
    execution.value = []
    await Promise.all([saveStaging(), saveExecution()])
  }

  function toggleAllCostmaps() {
    allCostmapToggle.value = !allCostmapToggle.value
  }

  function setNavState(state: string) {
    navState.value = state
  }

  async function resetAll() {
    await waypointsAPI.clearStore()
    await fetchAll()
  }

  return {
    store,
    staging,
    execution,
    allCostmapToggle,
    navState,
    isNavigating,
    highlightedWaypoint,
    autonEnabled,
    teleopEnabled,
    purePursuitEnabled,
    pathRelaxationEnabled,
    pathInterpolationEnabled,
    stereoDetectorEnabled,
    imageDetectorEnabled,
    odomFormat,
    clickPoint,
    storeForMap,
    stagingForMap,
    executionForMap,
    fetchStore,
    fetchStaging,
    fetchExecution,
    fetchAll,
    addToStore,
    updateStore,
    removeFromStore,
    addToStaging,
    removeFromStaging,
    clearStaging,
    stageAllToExecution,
    saveStaging,
    addToExecution,
    removeFromExecution,
    saveExecution,
    stageToExecution,
    stageNext,
    clearExecution,
    unstageOne,
    unstageExecution,
    toggleAllCostmaps,
    setNavState,
    resetAll,
  }
})

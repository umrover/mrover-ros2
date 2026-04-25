<template>
<<<<<<< HEAD
  <div class="flex flex-row gap-2">
    <div class="flex flex-col gap-1" style="flex: 1;">
=======
  <div class="flex flex-col gap-2">
    <div class="flex flex-col gap-1">
      <span class="data-label">General</span>
>>>>>>> origin/main
      <FeedbackButton
        ref="autonCheckbox"
        class="w-full"
        data-testid="pw-auton-toggle"
        :name="'Autonomy Mode'"
        :checked="autonEnabled"
        :action="autonAction"
        @toggle="handleAutonToggle"
      />
      <FeedbackButton
        ref="teleopCheckbox"
        class="w-full"
        data-testid="pw-teleop-toggle"
        :name="'Teleop Controls'"
        :checked="teleopEnabled"
        :action="teleopAction"
        @toggle="handleTeleopToggle"
      />
    </div>
<<<<<<< HEAD
    <div class="flex flex-col gap-1" style="flex: 1;">
=======
    <div class="flex flex-col gap-1">
      <span class="data-label">Costmap</span>
      <div class="flex gap-1">
        <button
          class="btn btn-sm flex-1 btn-success"
          data-testid="pw-costmap-all-on"
          @click="autonomyStore.setAllCostmaps(true)"
        >All On</button>
        <button
          class="btn btn-sm flex-1 btn-danger"
          data-testid="pw-costmap-all-off"
          @click="autonomyStore.setAllCostmaps(false)"
        >All Off</button>
      </div>
    </div>
    <div class="flex flex-col gap-1">
      <span class="data-label">Navigation</span>
>>>>>>> origin/main
      <FeedbackButton
        class="w-full"
        data-testid="pw-pure-pursuit-toggle"
        :name="'Pure Pursuit'"
        :checked="purePursuitEnabled"
        :action="purePursuitAction"
        @toggle="handlePurePursuitToggle"
      />
      <FeedbackButton
        class="w-full"
        data-testid="pw-path-relaxation-toggle"
        :name="'Path Relaxation'"
        :checked="pathRelaxationEnabled"
        :action="pathRelaxationAction"
        @toggle="handlePathRelaxationToggle"
      />
      <FeedbackButton
        class="w-full"
        data-testid="pw-path-interpolation-toggle"
        :name="'Path Interpolation'"
        :checked="pathInterpolationEnabled"
        :action="pathInterpolationAction"
        @toggle="handlePathInterpolationToggle"
      />
    </div>
  </div>
</template>

<script lang="ts" setup>
import { computed } from 'vue'
import FeedbackButton from './FeedbackButton.vue'
import { useAutonomyStore } from '@/stores/autonomy'
import { autonAPI } from '@/utils/api'

const emit = defineEmits<{
  toggleTeleop: [enabled: boolean]
}>()

<<<<<<< HEAD
=======
const autonAction = (newState: boolean) => {
  const waypoints = newState
    ? autonomyStore.execution.map(wp => ({
        latitude_degrees: wp.lat,
        longitude_degrees: wp.lon,
        tag_id: wp.tag_id,
        type: wp.type,
        enable_costmap: wp.enable_costmap,
      }))
    : []

  return autonAPI.enable(newState, waypoints)
}

>>>>>>> origin/main
const autonomyStore = useAutonomyStore()

const autonEnabled = computed(() => autonomyStore.autonEnabled)
const teleopEnabled = computed(() => autonomyStore.teleopEnabled)
const purePursuitEnabled = computed(() => autonomyStore.purePursuitEnabled)
const pathRelaxationEnabled = computed(() => autonomyStore.pathRelaxationEnabled)
const pathInterpolationEnabled = computed(() => autonomyStore.pathInterpolationEnabled)
<<<<<<< HEAD

const autonAction = (newState: boolean) => {
  const routeMap = autonomyStore.routeForMap
  const waypoints = newState
    ? routeMap.map((waypoint) => ({
        latitude_degrees: waypoint.latLng.lat,
        longitude_degrees: waypoint.latLng.lng,
        tag_id: waypoint.tag_id,
        type: waypoint.type,
        enable_costmap: waypoint.enable_costmap,
      }))
    : []

  return autonAPI.enable(newState, waypoints)
}
=======
const teleopAction = (newState: boolean) => autonAPI.enableTeleop(newState)
const purePursuitAction = (newState: boolean) => autonAPI.togglePurePursuit(newState)
const pathRelaxationAction = (newState: boolean) => autonAPI.togglePathRelaxation(newState)
const pathInterpolationAction = (newState: boolean) => autonAPI.togglePathInterpolation(newState)
>>>>>>> origin/main

const handleAutonToggle = (newState: boolean) => {
  autonomyStore.autonEnabled = newState
}

<<<<<<< HEAD
const teleopAction = (newState: boolean) => {
  return autonAPI.enableTeleop(newState)
}

=======
>>>>>>> origin/main
const handleTeleopToggle = (newState: boolean) => {
  autonomyStore.teleopEnabled = newState
  emit('toggleTeleop', newState)
}

<<<<<<< HEAD
const purePursuitAction = (newState: boolean) => {
  return autonAPI.togglePurePursuit(newState)
}

=======
>>>>>>> origin/main
const handlePurePursuitToggle = (newState: boolean) => {
  autonomyStore.purePursuitEnabled = newState
}

<<<<<<< HEAD
const pathRelaxationAction = (newState: boolean) => {
  return autonAPI.togglePathRelaxation(newState)
}

=======
>>>>>>> origin/main
const handlePathRelaxationToggle = (newState: boolean) => {
  autonomyStore.pathRelaxationEnabled = newState
}

<<<<<<< HEAD
const pathInterpolationAction = (newState: boolean) => {
  return autonAPI.togglePathInterpolation(newState)
}

=======
>>>>>>> origin/main
const handlePathInterpolationToggle = (newState: boolean) => {
  autonomyStore.pathInterpolationEnabled = newState
}
</script>
<<<<<<< HEAD

=======
>>>>>>> origin/main

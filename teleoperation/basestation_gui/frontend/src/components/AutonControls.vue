<template>
  <div class="flex flex-row gap-2">
    <div class="flex flex-col gap-1" style="flex: 1;">
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
    <div class="flex flex-col gap-1" style="flex: 1;">
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

const autonomyStore = useAutonomyStore()

const autonEnabled = computed(() => autonomyStore.autonEnabled)
const teleopEnabled = computed(() => autonomyStore.teleopEnabled)
const purePursuitEnabled = computed(() => autonomyStore.purePursuitEnabled)
const pathRelaxationEnabled = computed(() => autonomyStore.pathRelaxationEnabled)
const pathInterpolationEnabled = computed(() => autonomyStore.pathInterpolationEnabled)

const autonAction = (newState: boolean) => {
  const routeMap = autonomyStore.routeForMap
  const waypoints = newState
    ? routeMap.map((waypoint) => ({
        latitude_degrees: waypoint.latLng.lat,
        longitude_degrees: waypoint.latLng.lng,
        tag_id: waypoint.tag_id,
        type: waypoint.type,
        enable_costmap: waypoint.enable_costmap,
        coverage_radius: waypoint.coverage_radius,
      }))
    : []

  return autonAPI.enable(newState, waypoints)
}

const handleAutonToggle = (newState: boolean) => {
  autonomyStore.autonEnabled = newState
}

const teleopAction = (newState: boolean) => {
  return autonAPI.enableTeleop(newState)
}

const handleTeleopToggle = (newState: boolean) => {
  autonomyStore.teleopEnabled = newState
  emit('toggleTeleop', newState)
}

const purePursuitAction = (newState: boolean) => {
  return autonAPI.togglePurePursuit(newState)
}

const handlePurePursuitToggle = (newState: boolean) => {
  autonomyStore.purePursuitEnabled = newState
}

const pathRelaxationAction = (newState: boolean) => {
  return autonAPI.togglePathRelaxation(newState)
}

const handlePathRelaxationToggle = (newState: boolean) => {
  autonomyStore.pathRelaxationEnabled = newState
}

const pathInterpolationAction = (newState: boolean) => {
  return autonAPI.togglePathInterpolation(newState)
}

const handlePathInterpolationToggle = (newState: boolean) => {
  autonomyStore.pathInterpolationEnabled = newState
}
</script>


<template>
  <div class="d-flex flex-row gap-2">
    <div class="d-flex flex-column gap-1" style="flex: 1;">
      <FeedbackButton
        ref="autonCheckbox"
        class="w-100"
        :name="'Autonomy Mode'"
        :checked="autonEnabled"
        :action="autonAction"
        @toggle="handleAutonToggle"
      />
      <FeedbackButton
        ref="teleopCheckbox"
        class="w-100"
        data-testid="pw-teleop-toggle"
        :name="'Teleop Controls'"
        :checked="teleopEnabled"
        :action="teleopAction"
        @toggle="handleTeleopToggle"
      />
    </div>
    <div class="d-flex flex-column gap-1" style="flex: 1;">
      <FeedbackButton
        class="w-100"
        :name="'Pure Pursuit'"
        :checked="purePursuitEnabled"
        :action="purePursuitAction"
        @toggle="handlePurePursuitToggle"
      />
      <FeedbackButton
        class="w-100"
        :name="'Path Relaxation'"
        :checked="pathRelaxationEnabled"
        :action="pathRelaxationAction"
        @toggle="handlePathRelaxationToggle"
      />
      <FeedbackButton
        class="w-100"
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
  const route = autonomyStore.route
  const waypoints = newState
    ? route.map((waypoint) => ({
        latitude_degrees: waypoint.latLng.lat,
        longitude_degrees: waypoint.latLng.lng,
        tag_id: waypoint.id,
        type: waypoint.type,
        enable_costmap: waypoint.enable_costmap,
      }))
    : []

  return autonAPI.enable(newState, waypoints)
}

const handleAutonToggle = (newState: boolean) => {
  autonomyStore.setAutonMode(newState)
}

const teleopAction = (newState: boolean) => {
  return autonAPI.enableTeleop(newState)
}

const handleTeleopToggle = (newState: boolean) => {
  autonomyStore.setTeleopMode(newState)
  emit('toggleTeleop', newState)
}

const purePursuitAction = (newState: boolean) => {
  return autonAPI.togglePurePursuit(newState)
}

const handlePurePursuitToggle = (newState: boolean) => {
  autonomyStore.setPurePursuit(newState)
}

const pathRelaxationAction = (newState: boolean) => {
  return autonAPI.togglePathRelaxation(newState)
}

const handlePathRelaxationToggle = (newState: boolean) => {
  autonomyStore.setPathRelaxation(newState)
}

const pathInterpolationAction = (newState: boolean) => {
  return autonAPI.togglePathInterpolation(newState)
}

const handlePathInterpolationToggle = (newState: boolean) => {
  autonomyStore.setPathInterpolation(newState)
}
</script>


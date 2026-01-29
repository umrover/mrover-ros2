<template>
  <div class="d-flex flex-row gap-2 h-100">
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
        :name="'Teleop Controls'"
        :checked="teleopEnabled"
        :action="teleopAction"
        @toggle="handleTeleopToggle"
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
</script>


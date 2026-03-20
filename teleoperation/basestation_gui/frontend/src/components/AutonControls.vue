<template>
  <div class="grid grid-cols-3 gap-2">
    <div class="flex flex-col gap-1">
      <span class="cmd-data-label">General</span>
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
    <div class="flex flex-col gap-1">
      <span class="cmd-data-label">Navigation</span>
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
    <div class="flex flex-col gap-1">
      <span class="cmd-data-label">Perception</span>
      <FeedbackButton
        class="w-full"
        :name="'Stereo Detector'"
        :checked="stereoDetectorEnabled"
        :action="stereoDetectorAction"
        @toggle="handleStereoDetectorToggle"
      />
      <FeedbackButton
        class="w-full"
        :name="'Image Detector'"
        :checked="imageDetectorEnabled"
        :action="imageDetectorAction"
        @toggle="handleImageDetectorToggle"
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

const autonAction = (newState: boolean) => {
  const routeMap = autonomyStore.routeForMap
  const waypoints = newState
    ? routeMap.map(waypoint => ({
        latitude_degrees: waypoint.latLng.lat,
        longitude_degrees: waypoint.latLng.lng,
        tag_id: waypoint.tag_id,
        type: waypoint.type,
        enable_costmap: waypoint.enable_costmap,
      }))
    : []

  return autonAPI.enable(newState, waypoints)
}

const autonomyStore = useAutonomyStore()

const autonEnabled = computed(() => autonomyStore.autonEnabled)
const teleopEnabled = computed(() => autonomyStore.teleopEnabled)
const purePursuitEnabled = computed(() => autonomyStore.purePursuitEnabled)
const pathRelaxationEnabled = computed(() => autonomyStore.pathRelaxationEnabled)
const pathInterpolationEnabled = computed(() => autonomyStore.pathInterpolationEnabled)
const stereoDetectorEnabled = computed(() => autonomyStore.stereoDetectorEnabled)
const imageDetectorEnabled = computed(() => autonomyStore.imageDetectorEnabled)

const teleopAction = (newState: boolean) => autonAPI.enableTeleop(newState)
const purePursuitAction = (newState: boolean) => autonAPI.togglePurePursuit(newState)
const pathRelaxationAction = (newState: boolean) => autonAPI.togglePathRelaxation(newState)
const pathInterpolationAction = (newState: boolean) => autonAPI.togglePathInterpolation(newState)
const stereoDetectorAction = (newState: boolean) => autonAPI.toggleStereoDetector(newState)
const imageDetectorAction = (newState: boolean) => autonAPI.toggleImageDetector(newState)

const handleAutonToggle = (newState: boolean) => {
  autonomyStore.autonEnabled = newState
}

const handleTeleopToggle = (newState: boolean) => {
  autonomyStore.teleopEnabled = newState
  emit('toggleTeleop', newState)
}

const handlePurePursuitToggle = (newState: boolean) => {
  autonomyStore.purePursuitEnabled = newState
}

const handlePathRelaxationToggle = (newState: boolean) => {
  autonomyStore.pathRelaxationEnabled = newState
}

const handlePathInterpolationToggle = (newState: boolean) => {
  autonomyStore.pathInterpolationEnabled = newState
}

const handleStereoDetectorToggle = (newState: boolean) => {
  autonomyStore.stereoDetectorEnabled = newState
}

const handleImageDetectorToggle = (newState: boolean) => {
  autonomyStore.imageDetectorEnabled = newState
}
</script>

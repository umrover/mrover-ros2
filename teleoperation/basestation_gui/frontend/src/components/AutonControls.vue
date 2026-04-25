<template>
  <div class="flex flex-col gap-2">
    <div class="flex flex-col gap-1">
      <span class="data-label">General</span>
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
  const waypoints = newState
    ? autonomyStore.execution.map(wp => ({
        latitude_degrees: wp.lat,
        longitude_degrees: wp.lon,
        tag_id: wp.tag_id,
        type: wp.type,
        enable_costmap: wp.enable_costmap,
        coverage_radius: wp.coverage_radius,
      }))
    : []

  return autonAPI.enable(newState, waypoints)
}

const teleopAction = (newState: boolean) => autonAPI.enableTeleop(newState)
const purePursuitAction = (newState: boolean) => autonAPI.togglePurePursuit(newState)
const pathRelaxationAction = (newState: boolean) => autonAPI.togglePathRelaxation(newState)
const pathInterpolationAction = (newState: boolean) => autonAPI.togglePathInterpolation(newState)

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
</script>

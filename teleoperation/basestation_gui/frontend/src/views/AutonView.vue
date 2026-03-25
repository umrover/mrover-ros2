<template>
  <BaseGridView
    layout-key="autonView_gridLayout"
    :default-layout="defaultLayout"
    :topics="['drive', 'nav', 'science', 'chassis']"
  >
    <template #odometry>
      <div class="island p-2 rounded h-full">
        <OdometryReading />
      </div>
    </template>

    <template #nav-status>
      <div class="island p-2 rounded h-full flex flex-col">
        <NavigationStatus />
      </div>
    </template>

    <template #controls>
      <div class="island p-2 rounded h-full">
        <AutonControls @toggleTeleop="teleopEnabledCheck = $event" />
      </div>
    </template>

    <template #map>
      <div class="island p-0 rounded h-full overflow-hidden">
        <AutonRoverMap />
      </div>
    </template>

    <template #waypoints>
      <div class="island p-2 rounded h-full overflow-y-auto">
        <AutonWaypointEditor />
      </div>
    </template>

    <template #drive-data>
      <div class="island p-2 rounded h-full">
        <DriveStatusIndicator />
      </div>
    </template>
  </BaseGridView>

  <div
    v-if="!autonEnabled && teleopEnabledCheck"
    v-show="false"
    class="driveControls"
  >
    <DriveControls />
    <GimbalControls />
  </div>
</template>

<script lang="ts" setup>
import { ref } from 'vue'
import BaseGridView from '@/components/BaseGridView.vue'
import AutonRoverMap from '@/components/AutonRoverMap.vue'
import AutonWaypointEditor from '@/components/AutonWaypointEditor.vue'
import AutonControls from '@/components/AutonControls.vue'
import OdometryReading from '@/components/OdometryReading.vue'
import NavigationStatus from '@/components/NavigationStatus.vue'
import DriveControls from '@/components/DriveControls.vue'
import GimbalControls from '@/components/GimbalControls.vue'
import DriveStatusIndicator from '@/components/DriveStatusIndicator.vue'
import { useAutonomyStore } from '@/stores/autonomy'
import { storeToRefs } from 'pinia'

const defaultLayout = [
  { x: 0, y: 0, w: 6, h: 2, i: 'odometry' },
  { x: 0, y: 2, w: 6, h: 4, i: 'nav-status' },
  { x: 0, y: 7, w: 2, h: 5, i: 'controls' },
  { x: 0, y: 11, w: 2, h: 1, i: 'drive-data' },
  { x: 6, y: 0, w: 6, h: 6, i: 'map' },
  { x: 2, y: 6, w: 10, h: 6, i: 'waypoints' },
]

const autonomyStore = useAutonomyStore()
const { autonEnabled } = storeToRefs(autonomyStore)

const teleopEnabledCheck = ref(false)
</script>

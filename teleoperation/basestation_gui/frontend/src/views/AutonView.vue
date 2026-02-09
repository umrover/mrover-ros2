<template>
  <BaseGridView
    layout-key="autonView_gridLayout"
    :default-layout="defaultLayout"
    :topics="['drive', 'nav', 'science', 'chassis']"
  >
    <template #odometry>
      <div class="island p-2 rounded h-100">
        <OdometryReading />
      </div>
    </template>

    <template #nav-status>
      <div class="island p-2 rounded h-100 d-flex flex-column">
        <NavigationStatus />
      </div>
    </template>

    <template #controls>
      <div class="island p-2 rounded h-100">
        <AutonControls
          @toggleTeleop="teleopEnabledCheck = $event"
        />
      </div>
    </template>

    <template #velocity>
      <div class="island p-2 rounded h-100">
        <VelocityReading />
      </div>
    </template>

    <template #map>
      <div class="island p-0 rounded h-100 overflow-hidden">
        <AutonRoverMap />
      </div>
    </template>

    <template #waypoints>
      <div class="island p-2 rounded h-100 overflow-y-auto">
        <AutonWaypointEditor />
      </div>
    </template>

    <template #moteus>
      <div class="island p-2 rounded h-100 overflow-hidden">
        <ControllerDataTable mode="drive" header="Drive" />
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
import VelocityReading from '@/components/VelocityReading.vue'
import DriveControls from '@/components/DriveControls.vue'
import GimbalControls from '@/components/GimbalControls.vue'
import ControllerDataTable from '@/components/ControllerDataTable.vue'
import { useAutonomyStore } from '@/stores/autonomy'
import { storeToRefs } from 'pinia'

const defaultLayout = [
  { x: 0, y: 0, w: 6, h: 2, i: 'odometry' },
  { x: 0, y: 2, w: 6, h: 4, i: 'nav-status' },
  { x: 0, y: 7, w: 2, h: 2, i: 'velocity' },
  { x: 2, y: 7, w: 4, h: 2, i: 'controls' },
  { x: 0, y: 8, w: 6, h: 4, i: 'moteus' },
  { x: 6, y: 0, w: 6, h: 5, i: 'map' },
  { x: 6, y: 5, w: 6, h: 7, i: 'waypoints' },
]

const autonomyStore = useAutonomyStore()
const { autonEnabled } = storeToRefs(autonomyStore)

const teleopEnabledCheck = ref(false)
</script>

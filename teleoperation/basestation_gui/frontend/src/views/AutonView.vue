<template>
  <BaseGridView
    layout-key="autonView_gridLayout"
    :default-layout="defaultLayout"
    :topics="['arm', 'drive', 'nav', 'science', 'chassis']"
  >
    <template #odometry>
      <div class="island p-2 rounded h-full">
        <OdometryReading />
      </div>
<<<<<<< HEAD
    </template>
=======
  </template>
>>>>>>> origin/main

    <template #nav-status>
      <div class="island p-2 rounded h-full flex flex-col">
        <NavigationStatus />
      </div>
    </template>

<<<<<<< HEAD
    <template #controls>
      <div class="island p-2 rounded h-full">
        <AutonControls
          @toggleTeleop="teleopEnabledCheck = $event"
        />
      </div>
    </template>

    <template #velocity>
      <div class="island p-2 rounded h-full">
        <VelocityReading />
=======
    <template #rover-3d>
      <div class="island rounded overflow-hidden h-full">
        <Rover3D class="w-full h-full" />
      </div>
    </template>

    <template #controls>
      <div class="island p-2 rounded h-full">
        <AutonControls @toggleTeleop="teleopEnabledCheck = $event" />
>>>>>>> origin/main
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

<<<<<<< HEAD
    <template #arm-data>
      <div class="island p-2 rounded h-full">
        <ArmDataTable />
      </div>
    </template>

    <template #drive-data>
      <div class="island p-2 rounded h-full">
        <DriveDataTable />
=======
    <template #drive-data>
      <div class="island p-2 rounded h-full">
        <DriveStatusIndicator />
>>>>>>> origin/main
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
<<<<<<< HEAD
import ArmDataTable from '@/components/ControllerDataTable/ArmDataTable.vue'
import DriveDataTable from '@/components/ControllerDataTable/DriveDataTable.vue'
=======
import DriveStatusIndicator from '@/components/DriveStatusIndicator.vue'
import Rover3D from '@/components/Rover3D.vue'
>>>>>>> origin/main
import { useAutonomyStore } from '@/stores/autonomy'
import { storeToRefs } from 'pinia'

const defaultLayout = [
<<<<<<< HEAD
  { x: 0, y: 0, w: 6, h: 2, i: 'odometry' },
  { x: 0, y: 2, w: 6, h: 4, i: 'nav-status' },
  { x: 0, y: 7, w: 2, h: 2, i: 'velocity' },
  { x: 2, y: 7, w: 4, h: 2, i: 'controls' },
  { x: 0, y: 8, w: 3, h: 4, i: 'arm-data' },
  { x: 3, y: 8, w: 3, h: 4, i: 'drive-data' },
  { x: 6, y: 0, w: 6, h: 5, i: 'map' },
  { x: 6, y: 5, w: 6, h: 7, i: 'waypoints' },
=======
  { x: 0, y: 0, w: 5, h: 2, i: 'odometry' },
  { x: 0, y: 2, w: 5, h: 5, i: 'rover-3d' },
  { x: 3, y: 7, w: 3, h: 5, i: 'controls' },
  { x: 0, y: 7, w: 3, h: 4, i: 'nav-status' },
  { x: 0, y: 11, w: 3, h: 1, i: 'drive-data' },
  { x: 5, y: 0, w: 7, h: 7, i: 'map' },
  { x: 6, y: 7, w: 6, h: 5, i: 'waypoints' },
>>>>>>> origin/main
]

const autonomyStore = useAutonomyStore()
const { autonEnabled } = storeToRefs(autonomyStore)

const teleopEnabledCheck = ref(false)
</script>

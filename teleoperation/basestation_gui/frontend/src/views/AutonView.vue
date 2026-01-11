<template>
  <BaseGridView
    layout-key="autonView_gridLayout"
    :default-layout="defaultLayout"
    :topics="['drive', 'nav', 'science', 'chassis']"
  >
    <template #data>
      <div class="island p-2 rounded d-flex flex-column gap-2 h-100">
        <div class="rounded p-2 border border-2">
          <OdometryReading />
        </div>
        <NavigationStatus />
      </div>
    </template>

    <template #map>
      <div class="island p-0 rounded h-100 overflow-hidden">
        <AutonRoverMap />
      </div>
    </template>

    <template #waypoints>
      <div class="island p-2 rounded h-100 overflow-auto">
        <AutonWaypointEditor @toggleTeleop="teleopEnabledCheck = $event" />
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
import OdometryReading from '@/components/OdometryReading.vue'
import NavigationStatus from '@/components/NavigationStatus.vue'
import DriveControls from '@/components/DriveControls.vue'
import GimbalControls from '@/components/GimbalControls.vue'
import ControllerDataTable from '@/components/ControllerDataTable.vue'
import { useAutonomyStore } from '@/stores/autonomy'
import { storeToRefs } from 'pinia'

const defaultLayout = [
  { x: 0, y: 0, w: 6, h: 8, i: 'data' },
  { x: 0, y: 6, w: 6, h: 4, i: 'moteus' },
  { x: 6, y: 0, w: 6, h: 5, i: 'map' },
  { x: 6, y: 6, w: 6, h: 7, i: 'waypoints' },
]

const autonomyStore = useAutonomyStore()
const { autonEnabled } = storeToRefs(autonomyStore)

const teleopEnabledCheck = ref(false)
</script>

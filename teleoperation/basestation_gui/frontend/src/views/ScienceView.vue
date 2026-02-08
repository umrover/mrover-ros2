<template>
  <BaseGridView
    layout-key="scienceView_gridLayout"
    :default-layout="defaultLayout"
    :topics="['arm', 'chassis', 'drive', 'nav', 'science']"
  >
    <template #sparm>
      <div class="island p-2 rounded h-100">
        <SPArmControls class="h-100" />
      </div>
    </template>

    <template #gimbal>
      <div class="island p-2 rounded h-100">
        <GimbalControls class="h-100" />
      </div>
    </template>

    <template #drive>
      <div class="island p-2 rounded h-100">
        <DriveControls class="h-100" />
      </div>
    </template>

    <template #funnel>
      <div class="island p-2 rounded h-100">
        <FunnelControls class="h-100" />
      </div>
    </template>

    <template #panorama>
      <div class="island p-2 rounded h-100">
        <PanoramaControls class="h-100" />
      </div>
    </template>

    <template #odometry>
      <div class="island p-0 rounded h-100">
        <OdometryReading class="w-100 h-100" />
      </div>
    </template>

    <template #sensors>
      <div class="island p-2 rounded h-100 d-flex flex-column">
        <SensorData />
      </div>
    </template>

    <template #moteus>
      <div class="island p-2 rounded d-flex gap-2 h-100">
        <ControllerDataTable mode="drive" header="Drive" />
        <ControllerDataTable mode="sp" header="SP" />
      </div>
    </template>

    <template #map>
      <div class="island p-0 rounded overflow-hidden h-100">
        <BasicMap />
      </div>
    </template>

    <template #waypoints>
      <div class="island p-2 rounded h-100">
        <BasicWaypointEditor />
      </div>
    </template>
  </BaseGridView>
</template>

<script lang="ts" setup>
import BaseGridView from '@/components/BaseGridView.vue'
import SensorData from '@/components/SensorData.vue'
import BasicMap from '@/components/BasicRoverMap.vue'
import BasicWaypointEditor from '@/components/BasicWaypointEditor.vue'
import DriveControls from '@/components/DriveControls.vue'
import GimbalControls from '@/components/GimbalControls.vue'
import OdometryReading from '@/components/OdometryReading.vue'
import ControllerDataTable from '@/components/ControllerDataTable.vue'
import FunnelControls from '@/components/FunnelControls.vue'
import PanoramaControls from '@/components/PanoramaControls.vue'
import SPArmControls from '@/components/SPArmControls.vue'

const defaultLayout = [
  { x: 0, y: 0, w: 2, h: 2, i: 'sparm' },
  { x: 2, y: 0, w: 2, h: 2, i: 'gimbal' },
  { x: 4, y: 0, w: 2, h: 2, i: 'funnel' },
  { x: 6, y: 0, w: 1, h: 1, i: 'drive' },
  { x: 6, y: 1, w: 1, h: 1, i: 'panorama' },
  { x: 0, y: 3, w: 7, h: 2, i: 'odometry' },
  { x: 0, y: 5, w: 7, h: 4, i: 'sensors' },
  { x: 0, y: 9, w: 7, h: 4, i: 'moteus' },
  { x: 7, y: 0, w: 5, h: 6, i: 'map' },
  { x: 7, y: 6, w: 5, h: 6, i: 'waypoints' },
]
</script>

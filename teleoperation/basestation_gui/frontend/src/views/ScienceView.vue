<template>
  <BaseGridView
    layout-key="scienceView_gridLayout"
    :default-layout="defaultLayout"
    :topics="['arm', 'chassis', 'drive', 'nav', 'science']"
  >
    <template #controls>
      <div class="island p-2 rounded d-flex flex-row gap-1 h-100">
        <SPArmControls class="border border-2 p-1 rounded flex-fill" />
        <div class="d-flex flex-column gap-1">
          <GimbalControls class="border border-2 p-1 rounded flex-fill" />
          <DriveControls class="border border-2 p-1 rounded" />
        </div>
        <div class="d-flex flex-column gap-1">
          <FunnelControls class="border border-2 p-1 rounded flex-fill" />
          <PanoramaControls class="border border-2 p-1 rounded" />
        </div>
      </div>
    </template>

    <template #odometry>
      <div class="island p-2 rounded h-100">
        <OdometryReading class="rounded border border-2 w-100 h-100" />
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
      <div class="island p-1 rounded h-100">
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
  { x: 0, y: 0, w: 7, h: 3, i: 'controls' },
  { x: 0, y: 3, w: 7, h: 2, i: 'odometry' },
  { x: 0, y: 5, w: 7, h: 4, i: 'sensors' },
  { x: 0, y: 9, w: 7, h: 3, i: 'moteus' },
  { x: 7, y: 0, w: 5, h: 6, i: 'map' },
  { x: 7, y: 6, w: 5, h: 6, i: 'waypoints' },
]
</script>

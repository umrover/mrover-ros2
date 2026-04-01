<template>
  <BaseGridView
    layout-key="dmView_gridLayout"
    :default-layout="defaultLayout"
    :topics="['arm', 'drive', 'chassis', 'nav']"
  >
    <template #rover-3d>
      <div class="island rounded overflow-hidden h-full">
        <Rover3D class="w-full h-full" />
      </div>
    </template>

    <template #odometry>
      <div class="island p-2 rounded h-full">
        <OdometryReading />
      </div>
    </template>

    <template #arm-moteus>
      <div class="island p-2 rounded h-full">
        <ControllerDataTable mode="arm" header="Arm" />
      </div>
    </template>

    <template #drive-moteus>
      <div class="island p-2 rounded h-full">
        <ControllerDataTable mode="drive" header="Drive" />
      </div>
    </template>

    <template #arm-controls>
      <div class="island p-2 rounded h-full">
        <ArmControls />
      </div>
    </template>

    <template #gimbal-controls>
      <div class="island p-2 rounded h-full">
        <GimbalControls />
      </div>
    </template>

    <template #drive-controls>
      <div class="island p-2 rounded h-full">
        <DriveControls />
      </div>
    </template>

    <template #map>
      <div class="island p-0 rounded overflow-hidden h-full">
        <BasicMap />
      </div>
    </template>

    <template #waypoints>
      <div class="island p-2 rounded h-full">
        <BasicWaypointEditor :enableDrone="true" />
      </div>
    </template>
  </BaseGridView>
</template>

<script lang="ts" setup>
import BaseGridView from '@/components/BaseGridView.vue'
import ControllerDataTable from '@/components/ControllerDataTable.vue'
import ArmControls from '@/components/ArmControls.vue'
import BasicMap from '@/components/BasicRoverMap.vue'
import BasicWaypointEditor from '@/components/BasicWaypointEditor.vue'
import OdometryReading from '@/components/OdometryReading.vue'
import DriveControls from '@/components/DriveControls.vue'
import GimbalControls from '@/components/GimbalControls.vue'
import Rover3D from '@/components/Rover3D.vue'

const defaultLayout = [
  { x: 0, y: 0, w: 6, h: 4, i: 'rover-3d' },
  { x: 0, y: 4, w: 6, h: 2, i: 'odometry' },
  { x: 0, y: 6, w: 3, h: 2, i: 'arm-controls' },
  { x: 3, y: 6, w: 2, h: 2, i: 'gimbal-controls' },
  { x: 5, y: 6, w: 1, h: 2, i: 'drive-controls' },
  { x: 0, y: 8, w: 3, h: 4, i: 'arm-moteus' },
  { x: 3, y: 8, w: 3, h: 4, i: 'drive-moteus' },
  { x: 6, y: 0, w: 6, h: 6, i: 'map' },
  { x: 6, y: 6, w: 6, h: 6, i: 'waypoints' },
]
</script>

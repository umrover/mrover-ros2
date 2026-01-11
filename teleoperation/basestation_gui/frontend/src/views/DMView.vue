<template>
  <BaseGridView
    layout-key="dmView_gridLayout"
    :default-layout="defaultLayout"
    :topics="['arm', 'drive', 'chassis', 'nav']"
  >
    <template #rover-3d>
      <div class="island rounded overflow-hidden h-100">
        <Rover3D class="w-100 h-100" />
      </div>
    </template>

    <template #odometry>
      <div class="island p-2 rounded h-100">
        <OdometryReading />
      </div>
    </template>

    <template #moteus>
      <div class="island p-2 rounded d-flex flex-row gap-2 h-100">
        <ControllerDataTable mode="arm" header="Arm" />
        <ControllerDataTable mode="drive" header="Drive" />
      </div>
    </template>

    <template #controls>
      <div class="island p-2 rounded d-flex flex-row justify-content-between h-100">
        <ArmControls />
        <GimbalControls />
        <DriveControls />
      </div>
    </template>

    <template #map>
      <div class="island p-0 rounded overflow-hidden h-100">
        <BasicMap />
      </div>
    </template>

    <template #waypoints>
      <div class="island p-2 rounded h-100">
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
  { x: 0, y: 6, w: 6, h: 2, i: 'controls' },
  { x: 0, y: 8, w: 6, h: 4, i: 'moteus' },
  { x: 6, y: 0, w: 6, h: 6, i: 'map' },
  { x: 6, y: 6, w: 6, h: 6, i: 'waypoints' },
]
</script>

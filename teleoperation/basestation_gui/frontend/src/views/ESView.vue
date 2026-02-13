<template>
  <BaseGridView
    layout-key="esView_gridLayout"
    :default-layout="defaultLayout"
    :topics="['arm', 'drive']"
  >
    <template #arm-controls>
      <div class="island p-2 rounded h-full">
        <ArmControls />
      </div>
    </template>

    <template #drive-controls>
      <div class="island p-2 rounded h-full">
        <DriveControls />
      </div>
    </template>

    <template #rover-3d>
      <div class="island rounded overflow-hidden h-full">
        <Rover3D class="w-full h-full" />
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

    <template #auton-typing>
      <div class="island p-2 rounded h-full">
        <AutonTyping />
      </div>
    </template>
  </BaseGridView>
</template>

<script lang="ts" setup>
import BaseGridView from '@/components/BaseGridView.vue'
import ControllerDataTable from '@/components/ControllerDataTable.vue'
import ArmControls from '@/components/ArmControls.vue'
import DriveControls from '@/components/DriveControls.vue'
import Rover3D from '@/components/Rover3D.vue'
import AutonTyping from '@/components/AutonTyping.vue'

const defaultLayout = [
  { x: 0, y: 0, w: 3, h: 2, i: 'arm-controls' },
  { x: 3, y: 0, w: 1, h: 2, i: 'drive-controls' },
  { x: 0, y: 2, w: 4, h: 2, i: 'auton-typing' },
  { x: 0, y: 4, w: 4, h: 4, i: 'arm-moteus' },
  { x: 0, y: 8, w: 4, h: 4, i: 'drive-moteus' },
  { x: 4, y: 0, w: 8, h: 12, i: 'rover-3d' },
]
</script>

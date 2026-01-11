<template>
  <BaseGridView
    layout-key="esView_gridLayout"
    :default-layout="defaultLayout"
    :topics="['arm', 'drive']"
  >
    <template #arm-controls>
      <div class="island p-2 rounded h-100">
        <ArmControls />
      </div>
    </template>

    <template #rover-3d>
      <div class="island rounded overflow-hidden h-100">
        <Rover3D class="w-100 h-100" />
      </div>
    </template>

    <template #controller_state>
      <div class="island p-2 rounded d-flex flex-column gap-2 h-100">
        <ControllerDataTable mode="arm" header="Arm" />
        <ControllerDataTable mode="drive" header="Drive" />
      </div>
    </template>

    <template #auton-typing>
      <div class="island p-2 rounded h-100">
        <AutonTyping />
      </div>
    </template>
  </BaseGridView>
</template>

<script lang="ts" setup>
import BaseGridView from '@/components/BaseGridView.vue'
import ControllerDataTable from '@/components/ControllerDataTable.vue'
import ArmControls from '@/components/ArmControls.vue'
import Rover3D from '@/components/Rover3D.vue'
import AutonTyping from '@/components/AutonTyping.vue'

const defaultLayout = [
  { x: 0, y: 0, w: 4, h: 2, i: 'arm-controls' },
  { x: 0, y: 2, w: 4, h: 3, i: 'auton-typing' },
  { x: 0, y: 5, w: 4, h: 7, i: 'controller_state' },
  { x: 4, y: 0, w: 8, h: 12, i: 'rover-3d' },
]
</script>

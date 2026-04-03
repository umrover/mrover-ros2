<template>
  <div class="flex flex-col gap-2 h-full">
    <h4 class="component-header">Arm</h4>
    <div class="overflow-x-auto scroll flex-1">
      <table class="table compact-table w-full">
        <thead>
          <tr>
            <th>Joint</th>
            <th>State</th>
            <th>Err</th>
            <th>Pos</th>
            <th>Vel</th>
            <th>Cur</th>
            <th>Lim</th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="j in joints" :key="j.id" :class="stale ? 'row-no-data' : stateRowClass(stateFor(j.id), errorFor(j.id))">
            <td class="font-bold">{{ j.label }}</td>
            <td>{{ formatState(stateFor(j.id)) }}</td>
            <td>{{ formatError(errorFor(j.id)) }}</td>
            <td class="numeric-col"><span v-html="formatNumber(fieldAt(data.positions, j.id), 3, 2, true)"></span></td>
            <td class="numeric-col"><span v-html="formatNumber(fieldAt(data.velocities, j.id), 3, 2, true)"></span></td>
            <td class="numeric-col"><span v-html="formatNumber(fieldAt(data.currents, j.id), 3, 2, true)"></span></td>
            <td>{{ formatLimit(fieldAt(data.limitsHit, j.id)) }}</td>
          </tr>
        </tbody>
      </table>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { useControllerMessage, formatState, formatNumber, formatLimit, formatError, stateRowClass } from '@/composables/useControllerState'

const { stale, data } = useControllerMessage({
  topic: 'arm',
  messageType: 'arm_state',
})

const joints = [
  { id: 'gripper', label: 'Gripper' },
  { id: 'joint_de_roll', label: 'DE Roll' },
  { id: 'joint_de_pitch', label: 'DE Pitch' },
  { id: 'joint_c', label: 'Joint C' },
  { id: 'joint_b', label: 'Joint B' },
  { id: 'joint_a', label: 'Joint A' },
]

function indexFor(id: string): number {
  return data.value.names.indexOf(id)
}

function fieldAt<T>(arr: T[], id: string): T | undefined {
  const i = indexFor(id)
  return i >= 0 && i < arr.length ? arr[i] : undefined
}

function stateFor(id: string): string | undefined {
  return fieldAt(data.value.states, id)
}

function errorFor(id: string): string | undefined {
  return fieldAt(data.value.errors, id)
}
</script>

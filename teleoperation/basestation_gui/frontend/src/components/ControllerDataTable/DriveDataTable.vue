<template>
  <div class="flex flex-col gap-2 h-full">
    <h4 class="component-header">Drive</h4>
    <div class="overflow-x-auto cmd-scroll flex-1">
      <table class="cmd-table compact-table w-full">
        <thead>
          <tr>
            <th>Wheel</th>
            <th>State</th>
            <th>Err</th>
            <th>Vel</th>
            <th>Cur</th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="w in wheels" :key="w.id" :class="stale ? 'row-no-data' : stateRowClass(stateFor(w.id), errorFor(w.id))">
            <td class="font-bold">{{ w.label }}</td>
            <td>{{ formatState(stateFor(w.id)) }}</td>
            <td>{{ formatError(errorFor(w.id)) }}</td>
            <td class="numeric-col">{{ formatNumber(fieldAt(data.velocities, w.id)) }}</td>
            <td class="numeric-col">{{ formatNumber(fieldAt(data.currents, w.id)) }}</td>
          </tr>
        </tbody>
      </table>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { useControllerMessage, formatState, formatNumber, formatError, stateRowClass } from '@/composables/useControllerState'

const { stale, data } = useControllerMessage({
  topic: 'drive',
  messageType: 'drive_state',
})

const wheels = [
  { id: 'front_left', label: 'Front L' },
  { id: 'front_right', label: 'Front R' },
  { id: 'middle_left', label: 'Mid L' },
  { id: 'middle_right', label: 'Mid R' },
  { id: 'back_left', label: 'Rear L' },
  { id: 'back_right', label: 'Rear R' },
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

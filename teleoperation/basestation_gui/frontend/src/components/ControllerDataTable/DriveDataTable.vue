<template>
  <div class="flex flex-col gap-1 h-full">
    <div class="flex justify-between items-center pr-2 py-0">
      <h4 class="component-header">Drive State</h4>
      <button class="btn btn-icon-sm !h-6 !w-6 btn-outline-info" @click="legendModal?.open()">
        <i class="bi bi-info-circle"></i>
      </button>
    </div>
    <div class="overflow-x-auto scroll flex-1">
      <table class="table compact-table w-full">
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
            <td class="numeric-col"><span v-html="formatNumber(fieldAt(data.velocities, w.id), 3, 2, true)"></span></td>
            <td class="numeric-col"><span v-html="formatNumber(fieldAt(data.currents, w.id), 3, 2, true)"></span></td>
          </tr>
        </tbody>
      </table>
    </div>
    <StateMappingModal ref="legendModal" />
  </div>
</template>

<script lang="ts" setup>
import { ref } from 'vue'
import { useControllerMessage, formatState, formatNumber, formatError, stateRowClass } from '@/composables/useControllerState'
import StateMappingModal from '@/components/StateMappingModal.vue'

const legendModal = ref<InstanceType<typeof StateMappingModal> | null>(null)

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

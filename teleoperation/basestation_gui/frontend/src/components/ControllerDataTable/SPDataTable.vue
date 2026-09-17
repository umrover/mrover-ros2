<template>
  <div class="flex flex-col gap-1 h-full">
    <div class="flex justify-between items-center pr-2 py-0">
      <h4 class="component-header">Science State</h4>
      <button class="btn btn-icon-sm !h-6 !w-6 btn-outline-info" @click="legendModal?.open()">
        <i class="bi bi-info-circle"></i>
      </button>
    </div>
    <div class="overflow-x-auto scroll flex-1">
      <table class="table compact-table w-full">
        <thead>
          <tr>
            <th>Motor</th>
            <th>State</th>
            <th>Err</th>
            <th>Pos</th>
            <th>Cur</th>
            <th>Lim</th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="(name, i) in data.names" :key="i" :class="stale ? 'row-no-data' : stateRowClass(data.states[i], data.errors[i])">
            <td class="font-bold">{{ name }}</td>
            <td>{{ formatState(data.states[i]) }}</td>
            <td>{{ formatError(data.errors[i]) }}</td>
            <td class="numeric-col"><span v-html="formatNumber(data.positions[i], 3, 2, true)"></span></td>
            <td class="numeric-col"><span v-html="formatNumber(data.currents[i], 3, 2, true)"></span></td>
            <td>{{ formatLimit(data.limitsHit[i]) }}</td>
          </tr>
        </tbody>
      </table>
    </div>
    <StateMappingModal ref="legendModal" />
  </div>
</template>

<script lang="ts" setup>
import { ref } from 'vue'
import { useControllerMessage, formatState, formatNumber, formatLimit, formatError, stateRowClass } from '@/composables/useControllerState'
import StateMappingModal from '@/components/StateMappingModal.vue'

const legendModal = ref<InstanceType<typeof StateMappingModal> | null>(null)

const { stale, data } = useControllerMessage({
  topic: 'science',
  messageType: 'sp_controller_state',
})
</script>

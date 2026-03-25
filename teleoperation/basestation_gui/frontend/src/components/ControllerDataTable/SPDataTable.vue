<template>
  <div class="flex flex-col gap-2 h-full">
    <h4 class="component-header">SP</h4>
    <div class="overflow-x-auto cmd-scroll flex-1">
      <table class="cmd-table compact-table w-full">
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
            <td class="numeric-col">{{ formatNumber(data.positions[i]) }}</td>
            <td class="numeric-col">{{ formatNumber(data.currents[i]) }}</td>
            <td>{{ formatLimit(data.limitsHit[i]) }}</td>
          </tr>
        </tbody>
      </table>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { useControllerMessage, formatState, formatNumber, formatLimit, formatError, stateRowClass } from '@/composables/useControllerState'

const { stale, data } = useControllerMessage({
  storeKey: 'science',
  messageTypes: ['sp_controller_state'],
})
</script>

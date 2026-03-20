<template>
  <td class="fmt-num-tabular" :class="{ 'fmt-num-no-data': fmt.noData }">
    <span :class="{ 'fmt-num-pad': fmt.signDim }">{{ fmt.sign }}</span>
    <span class="fmt-num-pad">{{ fmt.pad }}</span>{{ fmt.num }}
  </td>
</template>

<script lang="ts" setup>
import { computed } from 'vue'
import { formatValue } from '@/utils/formatNumber'

const props = withDefaults(defineProps<{
  value: number | undefined
  intDigits?: number
  fracDigits?: number
}>(), {
  intDigits: 3,
  fracDigits: 2,
})

const fmt = computed(() => formatValue(props.value, props.intDigits, props.fracDigits))
</script>

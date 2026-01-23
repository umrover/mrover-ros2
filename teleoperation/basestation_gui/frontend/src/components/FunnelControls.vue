<template>
  <div class="d-flex flex-column align-items-center p-1">
    <div class="d-flex justify-content-between align-items-center w-100 mb-2">
      <h4 class="m-0">Funnel Controls</h4>
    </div>
    <div class="site-grid">
      <button
        v-for="(site, i) in sites"
        :key="i"
        class="btn btn-outline-control btn-sm"
        :class="{ active: currentSite === i }"
        @click="selectSite(i)"
      >
        {{ site }}
      </button>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref } from 'vue'

const emit = defineEmits<{
  selectSite: [site: number]
}>()

const sites = [
  'Griess B',
  'Sample',
  'Buret A',
  'Buret B',
  'Trash',
  'Griess A',
]
const currentSite = ref(0)

function selectSite(index: number) {
  currentSite.value = index
  emit('selectSite', index)
}
</script>

<style scoped>
.site-grid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 8px;
}

.btn {
  min-width: 100px;
}
</style>

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
        :class="{ active: currentSite === i, 'btn-warning': isLoading && pendingSite === i }"
        :disabled="isLoading"
        @click="selectSite(i)"
      >
        {{ site }}
      </button>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref } from 'vue'
import { scienceAPI } from '@/utils/api'

const sites = [
  'Griess B',
  'Sample',
  'Buret A',
  'Buret B',
  'Trash',
  'Griess A',
]

const site_to_radians: Record<number, number> = {
  0: 0.0,
  1: Math.PI / 3,
  2: (2 * Math.PI) / 3,
  3: Math.PI,
  4: (4 * Math.PI) / 3,
  5: (5 * Math.PI) / 3,
}

const currentSite = ref(0)
const pendingSite = ref<number | null>(null)
const isLoading = ref(false)

async function selectSite(index: number) {
  if (index === currentSite.value || isLoading.value) return

  const previousSite = currentSite.value
  currentSite.value = index
  pendingSite.value = index
  isLoading.value = true

  try {
    const radians = site_to_radians[index]
    if (radians !== undefined) {
      await scienceAPI.setGearDiffPosition(radians, false)
    }
  } catch {
    currentSite.value = previousSite
  } finally {
    isLoading.value = false
    pendingSite.value = null
  }
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

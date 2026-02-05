<template>
  <div class="d-flex flex-column align-items-center p-1">
    <div class="d-flex justify-content-between align-items-center w-100 mb-2">
      <h4 class="component-header">Funnel Controls</h4>
    </div>
    <div class="site-rows">
      <div class="site-row justify-content-center">
        <button
          class="btn btn-outline-control btn-sm border-2 site-btn"
          :class="{ active: currentSite === 1, 'btn-warning': isLoading && pendingSite === 1 }"
          :disabled="isLoading"
          @click="selectSite(1)"
        >
          Sample
        </button>
      </div>
      <div class="site-row justify-content-between">
        <button
          class="btn btn-outline-control btn-sm border-2 site-btn"
          :class="{ active: currentSite === 0, 'btn-warning': isLoading && pendingSite === 0 }"
          :disabled="isLoading"
          @click="selectSite(0)"
        >
          Griess B
        </button>
        <button
          class="btn btn-outline-control btn-sm border-2 site-btn"
          :class="{ active: currentSite === 2, 'btn-warning': isLoading && pendingSite === 2 }"
          :disabled="isLoading"
          @click="selectSite(2)"
        >
          Buret A
        </button>
      </div>
      <div class="site-row justify-content-between">
        <button
          class="btn btn-outline-control btn-sm border-2 site-btn"
          :class="{ active: currentSite === 3, 'btn-warning': isLoading && pendingSite === 3 }"
          :disabled="isLoading"
          @click="selectSite(3)"
        >
          Buret B
        </button>
        <button
          class="btn btn-outline-control btn-sm border-2 site-btn"
          :class="{ active: currentSite === 5, 'btn-warning': isLoading && pendingSite === 5 }"
          :disabled="isLoading"
          @click="selectSite(5)"
        >
          Griess A
        </button>
      </div>
      <div class="site-row justify-content-center">
        <button
          class="btn btn-outline-control btn-sm border-2 site-btn"
          :class="{ active: currentSite === 4, 'btn-warning': isLoading && pendingSite === 4 }"
          :disabled="isLoading"
          @click="selectSite(4)"
        >
          Trash
        </button>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref } from 'vue'
import { scienceAPI } from '@/utils/api'

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
.site-rows {
  display: flex;
  flex-direction: column;
  gap: 0.25rem;
  width: 100%;
}

.site-row {
  display: flex;
  gap: 0.25rem;
}

.site-btn {
  width: 70px;
  padding-top: 0.2rem;
  padding-bottom: 0.2rem;
}
</style>

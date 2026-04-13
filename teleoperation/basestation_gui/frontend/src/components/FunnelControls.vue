<template>
  <div class="flex flex-col items-center p-1" data-testid="pw-funnel-controls">
    <div class="flex justify-between items-center w-full mb-2">
      <h4 class="component-header">Funnel Controls</h4>
    </div>
    <div class="flex flex-col gap-1 w-full">
      <div class="flex gap-1 justify-center">
        <button
          class="btn btn-outline-control btn-sm w-[70px] py-[0.2rem]"
          :class="{ active: currentSite === 1, 'btn-warning': isLoading && pendingSite === 1 }"
          :disabled="isLoading"
          :data-testid="`pw-funnel-site-1`"
          @click="selectSite(1)"
        >
          Sample
        </button>
      </div>
      <div class="flex gap-1 justify-between">
        <button
          class="btn btn-outline-control btn-sm w-[70px] py-[0.2rem]"
          :class="{ active: currentSite === 0, 'btn-warning': isLoading && pendingSite === 0 }"
          :disabled="isLoading"
          :data-testid="`pw-funnel-site-0`"
          @click="selectSite(0)"
        >
          Griess B
        </button>
        <button
          class="btn btn-outline-control btn-sm w-[70px] py-[0.2rem]"
          :class="{ active: currentSite === 2, 'btn-warning': isLoading && pendingSite === 2 }"
          :disabled="isLoading"
          :data-testid="`pw-funnel-site-2`"
          @click="selectSite(2)"
        >
          Buret A
        </button>
      </div>
      <div class="flex gap-1 justify-between">
        <button
          class="btn btn-outline-control btn-sm w-[70px] py-[0.2rem]"
          :class="{ active: currentSite === 3, 'btn-warning': isLoading && pendingSite === 3 }"
          :disabled="isLoading"
          :data-testid="`pw-funnel-site-3`"
          @click="selectSite(3)"
        >
          Buret B
        </button>
        <button
          class="btn btn-outline-control btn-sm w-[70px] py-[0.2rem]"
          :class="{ active: currentSite === 5, 'btn-warning': isLoading && pendingSite === 5 }"
          :disabled="isLoading"
          :data-testid="`pw-funnel-site-5`"
          @click="selectSite(5)"
        >
          Griess A
        </button>
      </div>
      <div class="flex gap-1 justify-center">
        <button
          class="btn btn-outline-control btn-sm w-[70px] py-[0.2rem]"
          :class="{ active: currentSite === 4, 'btn-warning': isLoading && pendingSite === 4 }"
          :disabled="isLoading"
          :data-testid="`pw-funnel-site-4`"
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
  0: 4.2586,  // GRIESS B
  1: 3.1415,  // SAMPLE
  2: 2.0071,  // BURET A
  3: 5.1138,  // BURET B
  4: 0.0,     // TRASH
  5: 1.1693,  // GRIESS A
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


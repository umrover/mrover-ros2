<template>
  <div class="flex flex-col items-center p-1" data-testid="pw-funnel-controls">
    <div class="flex justify-between items-center w-full mb-2">
      <h4 class="component-header">Funnel</h4>
      <div class="flex items-center gap-1">
        <button class="cmd-btn cmd-btn-outline-control cmd-btn-sm px-1.5 h-5!" :disabled="isLoading" @click="adjustOffset(-1)">-</button>
        <span class="text-[10px] text-muted select-none">{{ offsetDeg }}°</span>
        <button class="cmd-btn cmd-btn-outline-control cmd-btn-sm px-1.5 h-5!" :disabled="isLoading" @click="adjustOffset(1)">+</button>
        <IndicatorDot :is-active="atTarget" />
      </div>
    </div>
    <div class="grid grid-cols-[auto_1fr_auto] flex-1 gap-1 w-full justify-items-center">
      <div class="flex flex-col gap-1 justify-center">
        <button
          v-for="site in [5, 4]"
          :key="site"
          class="cmd-btn cmd-btn-outline-control cmd-btn-sm w-17.5 py-[0.2rem]"
          :class="siteClass(site)"
          :disabled="isLoading"
          :data-testid="`pw-funnel-site-${site}`"
          @click="selectSite(site)"
        >
          {{ SITE_LABELS[site] }}
        </button>
      </div>
      <div class="flex flex-col justify-between items-center">
        <button
          class="cmd-btn cmd-btn-outline-control cmd-btn-sm w-17.5 py-[0.2rem]"
          :class="siteClass(0)"
          :disabled="isLoading"
          data-testid="pw-funnel-site-0"
          @click="selectSite(0)"
        >
          {{ SITE_LABELS[0] }}
        </button>
        <span class="cmd-data-value">{{ currentPositionDeg }}°</span>
        <button
          class="cmd-btn cmd-btn-outline-control cmd-btn-sm w-17.5 py-[0.2rem]"
          :class="siteClass(3)"
          :disabled="isLoading"
          data-testid="pw-funnel-site-3"
          @click="selectSite(3)"
        >
          {{ SITE_LABELS[3] }}
        </button>
      </div>
      <div class="flex flex-col gap-1 justify-center">
        <button
          v-for="site in [1, 2]"
          :key="site"
          class="cmd-btn cmd-btn-outline-control cmd-btn-sm w-17.5 py-[0.2rem]"
          :class="siteClass(site)"
          :disabled="isLoading"
          :data-testid="`pw-funnel-site-${site}`"
          @click="selectSite(site)"
        >
          {{ SITE_LABELS[site] }}
        </button>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, watch } from 'vue'
import { scienceAPI } from '@/utils/api'
import { useGamepad } from '@/composables/useGamepad'
import IndicatorDot from '@/components/IndicatorDot.vue'

const SITE_COUNT = 6
const LB_INDEX = 4
const RB_INDEX = 5
const PRESS_THRESHOLD = 0.5
const TWO_PI = 2 * Math.PI
const DEG_TO_RAD = Math.PI / 180
const RAD_TO_DEG = 180 / Math.PI
const OFFSET_STORAGE_KEY = 'funnel-offset-deg'

const SITE_RADIANS = [0, Math.PI / 3, (2 * Math.PI) / 3, Math.PI, (4 * Math.PI) / 3, (5 * Math.PI) / 3] as const
const SITE_LABELS = ['Cache', 'A Buret', 'A Griess', 'Trash', 'B Buret', 'B Griess'] as const

const confirmedSite = ref(0)
const pendingSite = ref<number | null>(null)
const isLoading = ref(false)
const atTarget = ref(false)
const offsetDeg = ref(Number(localStorage.getItem(OFFSET_STORAGE_KEY)) || 0)

const currentPositionDeg = computed(() =>
  Math.round((SITE_RADIANS[confirmedSite.value] ?? 0) * RAD_TO_DEG + offsetDeg.value)
)

function siteClass(site: number) {
  return {
    active: confirmedSite.value === site,
    'cmd-btn-secondary': isLoading.value && pendingSite.value === site,
  }
}

function resetOffset() {
  offsetDeg.value = 0
  localStorage.removeItem(OFFSET_STORAGE_KEY)
}

function persistOffset() {
  localStorage.setItem(OFFSET_STORAGE_KEY, String(offsetDeg.value))
}

async function adjustOffset(delta: number) {
  if (isLoading.value) return
  offsetDeg.value += delta
  const success = await sendPosition(confirmedSite.value)
  if (success) {
    persistOffset()
  }
}

const { buttons } = useGamepad({ controllerIdFilter: 'Microsoft', hz: 10 })

let prevLB = false
let prevRB = false

watch(buttons, (btns) => {
  const lb = (btns[LB_INDEX] ?? 0) > PRESS_THRESHOLD
  const rb = (btns[RB_INDEX] ?? 0) > PRESS_THRESHOLD
  if (lb && !prevLB) selectSite((confirmedSite.value - 1 + SITE_COUNT) % SITE_COUNT)
  if (rb && !prevRB) selectSite((confirmedSite.value + 1) % SITE_COUNT)
  prevLB = lb
  prevRB = rb
})

async function sendPosition(index: number): Promise<boolean> {
  if (isLoading.value) return false
  pendingSite.value = index
  isLoading.value = true
  try {
    const radians = (((SITE_RADIANS[index] ?? 0) + offsetDeg.value * DEG_TO_RAD) % TWO_PI + TWO_PI) % TWO_PI
    const result = await scienceAPI.setFunnelPosition(radians, false)
    atTarget.value = !!result.at_tgt
    confirmedSite.value = index
    return true
  } catch {
    atTarget.value = false
    resetOffset()
    return false
  } finally {
    isLoading.value = false
    pendingSite.value = null
  }
}

function selectSite(index: number) {
  if (index === confirmedSite.value) return
  sendPosition(index)
}
</script>

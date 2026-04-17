import { defineStore } from 'pinia'
import { ref, watch } from 'vue'

const LOCKED_KEY = 'gridLayout_locked'

export const useGridLayoutStore = defineStore('gridLayout', () => {
  const locked = ref(localStorage.getItem(LOCKED_KEY) !== 'false')
  const resetCounter = ref(0)

  watch(locked, (val) => {
    localStorage.setItem(LOCKED_KEY, String(val))
  })

  function toggleLock() {
    locked.value = !locked.value
  }

  function setLocked(value: boolean) {
    locked.value = value
  }

  function triggerReset() {
    resetCounter.value++
  }

  return {
    locked,
    resetCounter,
    toggleLock,
    setLocked,
    triggerReset,
  }
})

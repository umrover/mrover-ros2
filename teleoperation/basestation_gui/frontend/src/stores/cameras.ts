import { defineStore } from 'pinia'
import { ref } from 'vue'

export const useCamerasStore = defineStore('cameras', () => {
  // State
  const camsEnabled = ref<boolean[]>(new Array(9).fill(false))
  const names = ref<string[]>(Array.from({ length: 9 }, (_, i) => 'Camera: ' + i))
  const cameraIdx = ref(1)
  const cameraName = ref('')
  const capacity = ref(2)
  const qualities = ref<number[]>(new Array(9).fill(-1))
  const streamOrder = ref<number[]>([-1, -1, -1, -1])

  // Actions
  function setCamsEnabled(newCamsEnabled: boolean[]) {
    camsEnabled.value = newCamsEnabled
  }

  function setNames(newNames: string[]) {
    names.value = newNames
  }

  function setCameraIdx(newCameraIdx: number) {
    cameraIdx.value = newCameraIdx
  }

  function setCameraName(newCameraName: string) {
    cameraName.value = newCameraName
  }

  function setCapacity(newCapacity: number) {
    capacity.value = newCapacity
  }

  function setQualities(newQualities: number[]) {
    qualities.value = newQualities
  }

  function setStreamOrder(newStreamOrder: number[]) {
    streamOrder.value = newStreamOrder
  }

  return {
    // State
    camsEnabled,
    names,
    cameraIdx,
    cameraName,
    capacity,
    qualities,
    streamOrder,
    // Actions
    setCamsEnabled,
    setNames,
    setCameraIdx,
    setCameraName,
    setCapacity,
    setQualities,
    setStreamOrder
  }
})

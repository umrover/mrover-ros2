<template>
  <div class="funnel-container">
    <h4 class="m-0 font-monospace align-self-start">Funnel</h4>
    <div class="hex-ring">
      <div class="hex-center"></div>
      <button
        v-for="(site, i) in sites"
        :key="i"
        class="hex-btn"
        :class="{ active: currentSite === i }"
        :style="getButtonStyle(i)"
        @click="selectSite(i)"
      >
        {{ i + 1 }}
      </button>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref } from 'vue'

const emit = defineEmits<{
  selectSite: [site: number]
}>()

const sites = Array(6).fill(null)
const currentSite = ref(0)

const radius = 56

function getButtonStyle(index: number) {
  const angle = (index * 60 - 90) * (Math.PI / 180)
  const x = Math.cos(angle) * radius
  const y = Math.sin(angle) * radius
  return {
    transform: `translate(${x}px, ${y}px)`,
  }
}

function selectSite(index: number) {
  currentSite.value = index
  emit('selectSite', index)
}
</script>

<style scoped>
.funnel-container {
  display: flex;
  flex-direction: column;
  align-items: center;
  padding: 8px;
}


.hex-ring {
  position: relative;
  width: 160px;
  height: 160px;
  display: flex;
  align-items: center;
  justify-content: center;
}

.hex-center {
  width: 50px;
  height: 58px;
  background: #e9ecef;
  clip-path: polygon(50% 0%, 100% 25%, 100% 75%, 50% 100%, 0% 75%, 0% 25%);
}

.hex-btn {
  position: absolute;
  min-width: 40px;
  height: 32px;
  padding: 0 8px;
  border-radius: 4px;
  border: 2px solid #0d6efd;
  background: white;
  color: #0d6efd;
  font-size: 0.75rem;
  font-weight: 600;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: all 0.15s ease;
  white-space: nowrap;
}

.hex-btn:hover {
  background: #e7f1ff;
}

.hex-btn.active {
  background: #0d6efd;
  color: white;
}
</style>

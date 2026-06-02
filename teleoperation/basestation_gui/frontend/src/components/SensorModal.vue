<template>
  <Teleport to="body">
    <div
      class="modal-backdrop flex justify-center items-center"
      @click.self="$emit('close')"
    >
      <div class="sensor-modal-content panel">
        <div class="sensor-modal-header">
          <h4 class="component-header">All Sensor Charts</h4>
          <button class="btn btn-sm btn-outline-danger close-btn" @click="$emit('close')">
            <i class="bi bi-x-lg"></i>
          </button>
        </div>

        <div class="sensor-grid">
          <div class="sensor-title-cell">
            <span class="config-section-label">Display</span>
            <div class="config-row">
              <span class="config-label">Zero Baseline</span>
              <input type="checkbox" v-model="configBeginAtZero" class="config-checkbox" />
            </div>
            <div class="config-row">
              <span class="config-label">Smooth Lines</span>
              <input type="checkbox" v-model="configSmooth" class="config-checkbox" />
            </div>
            <div class="config-divider"></div>
            <button class="btn btn-sm btn-outline-danger w-full" @click="$emit('reset')">
              <i class="bi bi-arrow-counterclockwise"></i> Reset History
            </button>
          </div>

          <div
            v-for="(config, index) in chartConfigs"
            :key="index"
            class="sensor-chart-cell"
          >
            <div class="sensor-chart-cell-header">
              <span class="sensor-chart-title">{{ config.title }}</span>
              <div class="flex gap-1">
                <button class="btn btn-sm btn-outline-secondary" @click="downloadPNG(index)">
                  <i class="bi bi-download"></i> PNG
                </button>
                <button class="btn btn-sm btn-outline-secondary" @click="downloadCSV(index)">
                  <i class="bi bi-download"></i> CSV
                </button>
              </div>
            </div>
            <div class="sensor-chart-canvas">
              <canvas :id="`modal-chart-${index}`" style="width: 100%; height: 100%"></canvas>
            </div>
          </div>
        </div>
      </div>
    </div>
  </Teleport>
</template>

<script lang="ts" setup>
import { onMounted, onUnmounted, ref, watch } from 'vue'
import Chart from 'chart.js/auto'
import type { Chart as ChartType } from 'chart.js/auto'
import { currentTimestamp } from '@/utils/formatNumber'

interface ChartDataset {
  label: string
  color: string
  historyIndex: number
}

interface ChartConfig {
  title: string
  datasets: ChartDataset[]
}

interface Props {
  sensorHistory: number[][]
  timeCounter: number
}

interface Emits {
  close: []
  reset: []
}

const props = defineProps<Props>()
const emit = defineEmits<Emits>()

const chartConfigs: readonly ChartConfig[] = [
  {
    title: 'Oxygen (%)',
    datasets: [{ label: 'Oxygen', color: '#4D9DE0', historyIndex: 0 }],
  },
  {
    title: 'Humidity (%)',
    datasets: [{ label: 'Humidity', color: '#3BB273', historyIndex: 1 }],
  },
  {
    title: 'Temperature (C)',
    datasets: [{ label: 'Temp', color: '#E15554', historyIndex: 2 }],
  },
  {
    title: 'UV Index',
    datasets: [{ label: 'UV', color: '#7768AE', historyIndex: 3 }],
  },
  {
    title: 'Ozone (ppb)',
    datasets: [{ label: 'Ozone', color: '#F9A825', historyIndex: 4 }],
  },
  {
    title: 'CO₂ (Rel. %)',
    datasets: [{ label: 'CO₂', color: '#8D6E63', historyIndex: 5 }],
  },
  {
    title: 'Pressure (Pa)',
    datasets: [{ label: 'Pressure', color: '#26A69A', historyIndex: 6 }],
  },
] as const

const charts: (ChartType | null)[] = Array(chartConfigs.length).fill(null)

const configWindow = 30
const configBeginAtZero = ref(false)
const configSmooth = ref(false)

watch(configBeginAtZero, (val) => {
  for (const chart of charts) {
    const yScale = chart?.options.scales?.['y'] as { beginAtZero?: boolean } | undefined
    if (!yScale) continue
    yScale.beginAtZero = val
    chart!.update()
  }
})

watch(configSmooth, (val) => {
  for (const chart of charts) {
    if (!chart) continue
    chart.data.datasets.forEach(ds => {
      (ds as { tension: number }).tension = val ? 0.4 : 0.1
    })
    chart.update()
  }
})

const sanitizeFilename = (title: string): string => {
  return title.replace(/[^a-zA-Z0-9]+/g, '_').replace(/^_|_$/g, '')
}

const downloadPNG = (chartIndex: number): void => {
  const chart = charts[chartIndex]
  const config = chartConfigs[chartIndex]
  if (!chart || !config) return

  const canvas = chart.canvas
  const url = canvas.toDataURL('image/png')
  const anchor = document.createElement('a')
  anchor.href = url
  anchor.download = `${sanitizeFilename(config.title)}_${currentTimestamp()}.png`
  anchor.click()
}

const downloadCSV = (chartIndex: number): void => {
  const config = chartConfigs[chartIndex]
  if (!config || !config.datasets[0]) return

  const historyIndex = config.datasets[0].historyIndex
  const data = props.sensorHistory[historyIndex]
  if (!data) return

  let csv = `Time,${config.datasets[0].label}\n`
  for (let i = 0; i < data.length; i++) {
    csv += `${i},${data[i]}\n`
  }

  const anchor = document.createElement('a')
  anchor.href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(csv)
  anchor.download = `${sanitizeFilename(config.title)}_${currentTimestamp()}.csv`
  anchor.click()
}

const updateCharts = (): void => {
  const firstHistory = props.sensorHistory[0]
  if (!firstHistory) return

  const windowLen = Math.min(firstHistory.length, configWindow)

  for (let i = 0; i < chartConfigs.length; i++) {
    const chart = charts[i]
    const config = chartConfigs[i]
    if (!chart || !config) continue

    config.datasets.forEach((ds, idx) => {
      const dataset = chart.data.datasets[idx]
      const historyData = props.sensorHistory[ds.historyIndex]
      if (dataset && historyData) {
        dataset.data = historyData.slice(-windowLen)
      }
    })

    const startTime = Math.max(0, props.timeCounter - windowLen + 1)
    chart.data.labels = Array.from({ length: windowLen }, (_, i) => startTime + i)
    chart.update()
  }
}

const handleEscape = (event: KeyboardEvent): void => {
  if (event.key === 'Escape') {
    emit('close')
  }
}

onMounted(() => {
  window.addEventListener('keydown', handleEscape)

  const firstHistory = props.sensorHistory[0]
  if (!firstHistory) return

  for (let i = 0; i < chartConfigs.length; i++) {
    const canvasElement = document.getElementById(
      `modal-chart-${i}`,
    ) as HTMLCanvasElement | null

    const config = chartConfigs[i]
    if (!canvasElement || !config) continue

    const windowLen = Math.min(firstHistory.length, configWindow)
    const datasets = config.datasets.map(ds => {
      const historyData = props.sensorHistory[ds.historyIndex]
      return {
        label: ds.label,
        data: historyData ? historyData.slice(-windowLen) : [],
        fill: false,
        borderColor: ds.color,
        tension: configSmooth.value ? 0.4 : 0.1,
      }
    })

    const startTime = Math.max(0, props.timeCounter - windowLen + 1)

    charts[i] = new Chart(canvasElement, {
      type: 'line',
      data: {
        labels: Array.from({ length: windowLen }, (_, i) => startTime + i),
        datasets,
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        plugins: {
          legend: {
            display: config.datasets.length > 1,
            position: 'top',
          },
        },
        scales: {
          y: {
            beginAtZero: configBeginAtZero.value,
            title: {
              display: true,
              text: config.title,
              font: { size: 13 },
            },
            ticks: {
              maxTicksLimit: 10,
              precision: 2,
            },
          },
          x: {
            title: {
              display: true,
              text: 'Time (s)',
              font: { size: 13 },
            },
          },
        },
      },
    })
  }

  const interval = setInterval(updateCharts, 1000)

  onUnmounted(() => {
    window.removeEventListener('keydown', handleEscape)
    clearInterval(interval)
    charts.forEach(chart => chart?.destroy())
  })
})
</script>

<style scoped>
.modal-backdrop {
  position: fixed;
  top: 0;
  left: 0;
  z-index: 1000;
  width: 100%;
  height: 100%;
  background-color: var(--backdrop);
}

.sensor-modal-content {
  display: flex;
  flex-direction: column;
  width: 90%;
  max-width: 1400px;
  height: 90vh;
  overflow: hidden;
}

.sensor-modal-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  flex-shrink: 0;
  padding-bottom: 0.5rem;
  margin-bottom: 0.5rem;
  border-bottom: 2px solid var(--panel-border);
}

.close-btn {
  aspect-ratio: 1;
  padding: 0.25rem;
}

.sensor-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  grid-template-rows: repeat(4, 1fr);
  gap: 0.5rem;
  flex: 1;
  min-height: 0;
}

.sensor-title-cell {
  display: flex;
  flex-direction: column;
  gap: 0.5rem;
  padding: 0.75rem;
  border: var(--border-width) solid var(--panel-border);
  border-radius: var(--radius-sm);
}

.config-section-label {
  font-size: 0.65rem;
  font-weight: 700;
  text-transform: uppercase;
  letter-spacing: 0.07em;
  color: var(--text-muted);
}

.config-row {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 0.5rem;
}

.config-label {
  font-size: 0.75rem;
  color: var(--text-muted);
  white-space: nowrap;
}

.config-checkbox {
  width: 1rem;
  height: 1rem;
  cursor: pointer;
  flex-shrink: 0;
}

.config-divider {
  flex: 1;
}

.sensor-chart-cell {
  display: flex;
  flex-direction: column;
  min-height: 0;
  padding: 0.5rem;
  border: var(--border-width) solid var(--panel-border);
  border-radius: var(--radius-sm);
}

.sensor-chart-cell-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  flex-shrink: 0;
  margin-bottom: 0.25rem;
}

.sensor-chart-title {
  font-size: 0.75rem;
  font-weight: 600;
  color: var(--text-muted);
  text-transform: uppercase;
  letter-spacing: 0.03em;
  white-space: nowrap;
}

.sensor-chart-cell .btn {
  font-size: 0.6875rem;
}

.sensor-chart-canvas {
  flex: 1;
  min-height: 0;
  position: relative;
}
</style>

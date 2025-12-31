<template>
  <div
    class="modal-backdrop d-flex justify-content-center align-items-center"
    @click.self="$emit('close')"
  >
    <div
      class="bg-white rounded p-3"
      style="
        width: 90%;
        max-width: 1200px;
        height: 90vh;
        overflow: hidden;
        display: flex;
        flex-direction: column;
      "
    >
      <div class="d-flex justify-content-between align-items-center pb-2">
        <h4 class="mb-0">All Sensor Charts</h4>
        <div class="d-flex gap-2">
          <button class="btn btn-danger" @click="$emit('reset')">
            <i class="bi bi-arrow-counterclockwise"></i> Reset
          </button>
          <button class="btn btn-secondary" @click="$emit('close')">
            <i class="bi bi-x-lg"></i>
          </button>
        </div>
      </div>

      <div class="d-flex flex-column gap-2 flex-fill" style="overflow: hidden">
        <div
          v-for="(config, index) in chartConfigs"
          :key="index"
          class="border rounded p-2 bg-light d-flex flex-row gap-3"
          style="flex: 1; min-height: 0"
        >
          <div
            class="d-flex flex-column align-items-start"
            style="width: 180px; flex-shrink: 0"
          >
            <h5 class="mb-2 text-nowrap">{{ config.title }}</h5>
            <div class="d-flex flex-column gap-2">
              <button class="btn btn-primary" @click="downloadPNG(index)">
                <i class="bi bi-download"></i> PNG
              </button>
              <button class="btn btn-secondary" @click="downloadCSV(index)">
                <i class="bi bi-download"></i> CSV
              </button>
            </div>
          </div>
          <div class="flex-fill d-flex" style="min-width: 0">
            <canvas
              :id="`modal-chart-${index}`"
              style="width: 100%; height: 100%"
            ></canvas>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { onMounted, onUnmounted, watch } from 'vue'
import Chart from 'chart.js/auto'
import type { Chart as ChartType } from 'chart.js/auto'

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
    title: 'Temperature (°C)',
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
    title: 'CO₂ (ppm)',
    datasets: [{ label: 'CO₂', color: '#8D6E63', historyIndex: 5 }],
  },
  {
    title: 'Pressure (Pa)',
    datasets: [{ label: 'Pressure', color: '#26A69A', historyIndex: 6 }],
  },
] as const

const charts: (ChartType | null)[] = Array(chartConfigs.length).fill(null)

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
  anchor.download = `${sanitizeFilename(config.title)}.png`
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
  anchor.download = `${sanitizeFilename(config.title)}.csv`
  anchor.click()
}

const updateCharts = (): void => {
  const firstHistory = props.sensorHistory[0]
  if (!firstHistory) return

  for (let i = 0; i < chartConfigs.length; i++) {
    const chart = charts[i]
    const config = chartConfigs[i]
    if (!chart || !config) continue

    config.datasets.forEach((ds, idx) => {
      const dataset = chart.data.datasets[idx]
      const historyData = props.sensorHistory[ds.historyIndex]
      if (dataset && historyData) {
        dataset.data = [...historyData]
      }
    })

    const startTime = Math.max(0, props.timeCounter - firstHistory.length + 1)
    chart.data.labels = Array.from(
      { length: firstHistory.length },
      (_, i) => startTime + i,
    )
    chart.update()
  }
}

const handleEscape = (event: KeyboardEvent): void => {
  if (event.key === 'Escape') {
    emit('close')
  }
}

onMounted(() => {
  console.log()
  window.addEventListener('keydown', handleEscape)

  const firstHistory = props.sensorHistory[0]
  if (!firstHistory) return

  for (let i = 0; i < chartConfigs.length; i++) {
    const canvasElement = document.getElementById(
      `modal-chart-${i}`,
    ) as HTMLCanvasElement | null

    const config = chartConfigs[i]
    if (!canvasElement || !config) continue

    const datasets = config.datasets.map(ds => {
      const historyData = props.sensorHistory[ds.historyIndex]
      return {
        label: ds.label,
        data: historyData ? [...historyData] : [],
        fill: false,
        borderColor: ds.color,
        tension: 0.1,
      }
    })

    const startTime = Math.max(0, props.timeCounter - firstHistory.length + 1)

    charts[i] = new Chart(canvasElement, {
      type: 'line',
      data: {
        labels: Array.from(
          { length: firstHistory.length },
          (_, i) => startTime + i,
        ),
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
            beginAtZero: true,
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

watch(() => props.sensorHistory, updateCharts, { deep: true })
</script>

<style scoped>
.modal-backdrop {
  position: fixed;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  background-color: rgba(0, 0, 0, 0.7);
  z-index: 1000;
}
</style>

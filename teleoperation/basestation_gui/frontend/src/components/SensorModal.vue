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
              <button
                class="btn btn-primary"
                @click="downloadPNG(index)"
              >
                <i class="bi bi-download"></i> PNG
              </button>
              <button
                class="btn btn-secondary"
                @click="downloadCSV(index)"
              >
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

const props = defineProps<{
  sensorHistory: number[][]
  timeCounter: number
}>()

const chartConfigs = [
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
]

const charts: Chart[] = []

const sanitizeFilename = (title: string) => {
  return title.replace(/[^a-zA-Z0-9]+/g, '_').replace(/^_|_$/g, '')
}

const downloadPNG = (chartIndex: number) => {
  const chart = charts[chartIndex]
  if (!chart) return

  const canvas = chart.canvas
  const url = canvas.toDataURL('image/png')
  const anchor = document.createElement('a')
  anchor.href = url
  anchor.download = `${sanitizeFilename(chartConfigs[chartIndex].title)}.png`
  anchor.click()
}

const downloadCSV = (chartIndex: number) => {
  const config = chartConfigs[chartIndex]
  const historyIndex = config.datasets[0].historyIndex

  let csv = `Time,${config.datasets[0].label}\n`
  const data = props.sensorHistory[historyIndex]

  for (let i = 0; i < data.length; ++i) {
    csv += `${i},${data[i]}\n`
  }

  const anchor = document.createElement('a')
  anchor.href = 'data:text/csv;charset=utf-8,' + encodeURIComponent(csv)
  anchor.download = `${sanitizeFilename(config.title)}.csv`
  anchor.click()
}

const updateCharts = () => {
  for (let i = 0; i < chartConfigs.length; ++i) {
    const chart = charts[i]
    if (chart) {
      const config = chartConfigs[i]
      config.datasets.forEach((ds, idx) => {
        chart.data.datasets[idx].data = [
          ...props.sensorHistory[ds.historyIndex],
        ]
      })

      const startTime = Math.max(
        0,
        props.timeCounter - props.sensorHistory[0].length + 1,
      )
      chart.data.labels = Array.from(
        { length: props.sensorHistory[0].length },
        (_, i) => startTime + i,
      )
      chart.update()
    }
  }
}

const emit = defineEmits<{
  close: []
  reset: []
}>()

const handleEscape = (event: KeyboardEvent) => {
  if (event.key === 'Escape') {
    emit('close')
  }
}

onMounted(() => {
  window.addEventListener('keydown', handleEscape)

  for (let i = 0; i < chartConfigs.length; ++i) {
    const canvasElement = document.getElementById(
      `modal-chart-${i}`,
    ) as HTMLCanvasElement
    if (canvasElement) {
      const config = chartConfigs[i]
      const datasets = config.datasets.map(ds => ({
        label: ds.label,
        data: [...props.sensorHistory[ds.historyIndex]],
        fill: false,
        borderColor: ds.color,
        tension: 0.1,
      }))

      const startTime = Math.max(
        0,
        props.timeCounter - props.sensorHistory[0].length + 1,
      )

      charts[i] = new Chart(canvasElement, {
        type: 'line',
        data: {
          labels: Array.from(
            { length: props.sensorHistory[0].length },
            (_, i) => startTime + i,
          ),
          datasets: datasets,
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
  }

  const interval = setInterval(updateCharts, 1000)

  onUnmounted(() => {
    window.removeEventListener('keydown', handleEscape)
    clearInterval(interval)
    charts.forEach(chart => chart.destroy())
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

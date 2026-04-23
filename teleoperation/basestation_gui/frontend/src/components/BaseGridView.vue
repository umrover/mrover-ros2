<template>
  <div ref="wrapperRef" class="view-wrapper">
    <GridLayout
      v-model:layout="layout"
      :col-num="12"
      :row-height="rowHeight"
      :margin="[MARGIN, MARGIN]"
      :is-draggable="!locked"
      :is-resizable="!locked"
      :vertical-compact="true"
      :use-css-transforms="true"
      @layout-updated="saveLayout"
    >
      <GridItem
        v-for="item in layout"
        :key="item.i"
        :x="item.x"
        :y="item.y"
        :w="item.w"
        :h="item.h"
        :i="item.i"
      >
        <slot :name="item.i" :item="item" />
      </GridItem>
    </GridLayout>
  </div>
</template>

<script lang="ts" setup>
import { ref, watch, onMounted, onUnmounted, nextTick } from 'vue'
import { GridLayout, GridItem } from 'vue-grid-layout-v3'
import { useGridLayoutStore } from '@/stores/gridLayout'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'

export interface LayoutItem {
  x: number
  y: number
  w: number
  h: number
  i: string
}

const props = defineProps<{
  layoutKey: string
  defaultLayout: LayoutItem[]
  topics?: string[]
}>()

const GRID_ROWS = 12
const MARGIN = 10

const wrapperRef = ref<HTMLElement | null>(null)
const rowHeight = ref(30)

const calculateRowHeight = () => {
  if (!wrapperRef.value) return
  const availableHeight = wrapperRef.value.clientHeight
  const totalMargin = MARGIN * (GRID_ROWS + 1)
  rowHeight.value = Math.floor((availableHeight - totalMargin) / GRID_ROWS)
}

const loadLayout = (): LayoutItem[] => {
  const saved = localStorage.getItem(props.layoutKey)
  if (saved) {
    try {
      return JSON.parse(saved)
    } catch {
      return props.defaultLayout
    }
  }
  return props.defaultLayout
}

const layout = ref<LayoutItem[]>(loadLayout())

const saveLayout = () => {
  localStorage.setItem(props.layoutKey, JSON.stringify(layout.value))
}

const gridLayoutStore = useGridLayoutStore()
const { locked, resetCounter } = storeToRefs(gridLayoutStore)

const websocketStore = useWebsocketStore()

watch(resetCounter, () => {
  layout.value = JSON.parse(JSON.stringify(props.defaultLayout))
  saveLayout()
})

onMounted(() => {
  nextTick(() => {
    calculateRowHeight()
  })
  window.addEventListener('resize', calculateRowHeight)
  props.topics?.forEach(topic => websocketStore.setupWebSocket(topic))
})

onUnmounted(() => {
  window.removeEventListener('resize', calculateRowHeight)
  props.topics?.forEach(topic => websocketStore.closeWebSocket(topic))
})
</script>

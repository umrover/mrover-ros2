<template>
  <div
    class="h-full flex flex-col cursor-pointer select-none"
    @click="modal.show()"
  >
    <h4 class="component-header flex items-center justify-between">
      <span class="flex items-center gap-1 hover:underline">
        Drive Status
        <i class="bi bi-box-arrow-up-right text-xs"></i>
      </span>
      <IndicatorDot :is-active="allArmed" />
    </h4>
  </div>

  <Teleport to="body">
    <div v-if="modal.isOpen.value" class="modal-backdrop" @click.self="modal.hide()">
      <div class="modal-dialog" style="max-width: 600px;">
        <div class="modal-content">
          <div class="modal-header">
            <h5 class="modal-title">Drive Data</h5>
            <button class="btn-close" @click="modal.hide()">
              <i class="bi bi-x-lg"></i>
            </button>
          </div>
          <div class="modal-body">
            <DriveDataTable />
          </div>
        </div>
      </div>
    </div>
  </Teleport>
</template>

<script lang="ts" setup>
import { ref, computed, watch } from 'vue'
import { useWebsocketStore } from '@/stores/websocket'
import { storeToRefs } from 'pinia'
import type { ControllerStateMessage } from '@/types/websocket'
import { useModal } from '@/composables/useModal'
import IndicatorDot from '@/components/IndicatorDot.vue'
import DriveDataTable from '@/components/ControllerDataTable/DriveDataTable.vue'

const modal = useModal()
const websocketStore = useWebsocketStore()
const { messages } = storeToRefs(websocketStore)

const states = ref<string[]>([])

let leftState: ControllerStateMessage | null = null
let rightState: ControllerStateMessage | null = null

function combineStates() {
  const l = leftState?.states ?? []
  const r = rightState?.states ?? []
  states.value = [...l, ...r]
}

const driveMessage = computed(() => messages.value['drive'])

watch(driveMessage, (msg) => {
  if (!msg) return
  const typed = msg as ControllerStateMessage
  if (typed.type === 'drive_left_state') {
    leftState = typed
    combineStates()
  } else if (typed.type === 'drive_right_state') {
    rightState = typed
    combineStates()
  }
})

const allArmed = computed(() =>
  states.value.length > 0 && states.value.every((s) => s === 'ARMED')
)
</script>

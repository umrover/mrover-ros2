 <template>
  <div
    class="h-full flex flex-col cursor-pointer select-none"
    @click="modal.show()"
  >
    <h4 class="component-header flex items-center justify-between">
      <span class="flex items-center gap-1 hover:underline">
        Payload Status
        <i class="bi bi-box-arrow-up-right text-xs"></i>
      </span>
      <IndicatorDot :is-active="allArmed" />
    </h4>
  </div>

  <Teleport to="body">
    <div v-if="modal.isOpen.value" class="modal-backdrop" @click.self="modal.hide()">
      <div class="modal-dialog max-w-150">
        <div class="modal-content">
          <div class="modal-header">
            <h5 class="modal-title">Payload Data</h5>
            <button class="btn-close" @click="modal.hide()">
              <i class="bi bi-x-lg"></i>
            </button>
          </div>
          <div class="modal-body">
            <ArmDataTable />
          </div>
        </div>
      </div>
    </div>
  </Teleport>
</template>

<script lang="ts" setup>
import { computed } from 'vue'
import { useControllerMessage } from '@/composables/useControllerState'
// import type { ControllerStateMessage } from '@/types/websocket'
import { useModal } from '@/composables/useModal'
import IndicatorDot from '@/components/IndicatorDot.vue'
import ArmDataTable from '@/components/ControllerDataTable/ArmDataTable.vue'
// import onMessage from '@useWebsocketStore.vue'

const modal = useModal()

const { data } = useControllerMessage({
  topic: 'payload_controller_state',
  messageType: 'ControllerStateMessage',
})

const allArmed = computed(() =>
  data.value.states.length > 0 && data.value.states.every((s) => s === 'ARMED')
)

// TODO: Create websocket (remember starter project)

</script>

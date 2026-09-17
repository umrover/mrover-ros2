<template>
  <Teleport to="body">
    <div v-if="isOpen" class="modal-backdrop" @click.self="close">
      <div class="modal-dialog modal-lg">
        <div class="modal-content">
          <div class="modal-header">
            <h5 class="modal-title">Controller State Legend</h5>
            <button type="button" class="btn-close" @click="close">
              <i class="bi bi-x-lg"></i>
            </button>
          </div>
          <div class="modal-body max-h-[70vh] overflow-y-auto scroll">
            <div class="space-y-6 pt-2 pb-4">
              <!-- Moteus (Brushless) -->
              <section>
                <h6 class="text-base font-bold text-muted uppercase mb-4! border-b border-theme pb-2!">Moteus (Brushless)</h6>
                <div class="grid grid-cols-2 gap-x-8 gap-y-1">
                  <div v-for="(compact, full) in MOTEUS_MAP" :key="full" class="flex justify-between items-center py-1">
                    <span class="text-xs font-bold text-primary">{{ compact }}</span>
                    <span class="text-xs text-muted">{{ full }}</span>
                  </div>
                </div>
              </section>

              <!-- BMC (Brushed) -->
              <section>
                <h6 class="text-base font-bold text-muted uppercase mb-4! border-b border-theme pb-2!">BMC (Brushed)</h6>
                <div class="grid grid-cols-2 gap-x-8 gap-y-1">
                  <div v-for="(compact, full) in BMC_MAP" :key="full" class="flex justify-between items-center py-1">
                    <span class="text-xs font-bold text-primary">{{ compact }}</span>
                    <span class="text-xs text-muted">{{ full }}</span>
                  </div>
                </div>
              </section>

              <!-- U2D2 (Servos) -->
              <section>
                <h6 class="text-base font-bold text-muted uppercase mb-4! border-b border-theme pb-2!">U2D2 (Servos)</h6>
                <div class="grid grid-cols-2 gap-x-8 gap-y-1">
                  <div v-for="(compact, full) in SERVO_MAP" :key="full" class="flex justify-between items-center py-1">
                    <span class="text-xs font-bold text-primary">{{ compact }}</span>
                    <span class="text-xs text-muted">{{ full }}</span>
                  </div>
                </div>
              </section>

              <!-- Simulator -->
              <section>
                <h6 class="text-base font-bold text-muted uppercase mb-4! border-b border-theme pb-2!">Simulator</h6>
                <div class="grid grid-cols-2 gap-x-8 gap-y-1">
                  <div v-for="(compact, full) in SIM_MAP" :key="full" class="flex justify-between items-center py-1">
                    <span class="text-xs font-bold text-primary">{{ compact }}</span>
                    <span class="text-xs text-muted">{{ full }}</span>
                  </div>
                </div>
              </section>
            </div>
          </div>
          <div class="modal-footer">
            <button type="button" class="btn btn-secondary btn-sm" @click="close">Close</button>
          </div>
        </div>
      </div>
    </div>
  </Teleport>
</template>

<script lang="ts" setup>
import { ref } from 'vue'
import { MOTEUS_MAP, BMC_MAP, SERVO_MAP, SIM_MAP } from '@/composables/useControllerState'

const isOpen = ref(false)

function open() {
  isOpen.value = true
}

function close() {
  isOpen.value = false
}

defineExpose({ open })
</script>

<style scoped>
.modal-lg {
  max-width: 600px;
}
</style>

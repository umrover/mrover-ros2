<template>
  <Teleport to="body">
    <div class="modal fade" :id="modalId" tabindex="-1" role="dialog">
      <div class="modal-dialog modal-dialog-centered" role="document">
        <div class="modal-content">
          <div class="modal-header">
            <h5 class="modal-title">{{ title }}</h5>
            <button type="button" class="btn-close" @click="close"></button>
          </div>
          <div class="modal-body">
            <slot>
              <p class="mb-0">{{ message }}</p>
            </slot>
          </div>
          <div class="modal-footer">
            <button type="button" class="btn btn-secondary btn-sm border-2" @click="close">Cancel</button>
            <button
              type="button"
              class="btn btn-sm border-2"
              :class="`btn-${confirmVariant}`"
              @click="confirm"
            >{{ confirmText }}</button>
          </div>
        </div>
      </div>
    </div>
  </Teleport>
</template>

<script lang="ts" setup>
import { ref, onMounted } from 'vue'
import { Modal } from 'bootstrap'

const props = withDefaults(defineProps<{
  modalId: string
  title: string
  message?: string
  confirmText?: string
  confirmVariant?: string
}>(), {
  message: 'Are you sure?',
  confirmText: 'Confirm',
  confirmVariant: 'danger',
})

const emit = defineEmits<{
  confirm: []
}>()

const modal = ref<Modal | null>(null)

onMounted(() => {
  modal.value = new Modal(`#${props.modalId}`, {})
})

function open() {
  modal.value?.show()
}

function close() {
  modal.value?.hide()
}

function confirm() {
  emit('confirm')
  close()
}

defineExpose({ open, close })
</script>

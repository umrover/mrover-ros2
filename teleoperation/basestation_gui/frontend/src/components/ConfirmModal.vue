<template>
  <Teleport to="body">
    <div v-if="isOpen" class="modal-backdrop" @click.self="close">
      <div class="modal-dialog">
        <div class="modal-content">
          <div class="modal-header">
            <h5 class="modal-title">{{ title }}</h5>
            <button type="button" class="btn-close" @click="close"><i class="bi bi-x-lg"></i></button>
          </div>
          <div class="modal-body">
            <slot>
              <p class="mb-0">{{ message }}</p>
            </slot>
          </div>
          <div class="modal-footer">
            <button type="button" class="btn btn-secondary btn-sm" @click="close">Cancel</button>
            <button
              type="button"
              class="btn btn-sm"
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
import { useModal } from '@/composables/useModal'

const { isOpen, show, hide } = useModal()

withDefaults(defineProps<{
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

function open() { show() }
function close() { hide() }
function confirm() { emit('confirm'); hide() }

defineExpose({ open, close })
</script>

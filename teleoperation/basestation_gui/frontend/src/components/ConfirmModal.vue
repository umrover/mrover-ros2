<template>
  <Teleport to="body">
    <div v-if="isOpen" class="cmd-modal-backdrop" @click.self="close">
      <div class="cmd-modal-dialog">
        <div class="cmd-modal-content">
          <div class="cmd-modal-header">
            <h5 class="cmd-modal-title">{{ title }}</h5>
            <button type="button" class="cmd-btn-close" @click="close"><i class="bi bi-x-lg"></i></button>
          </div>
          <div class="cmd-modal-body">
            <slot>
              <p class="mb-0">{{ message }}</p>
            </slot>
          </div>
          <div class="cmd-modal-footer">
            <button type="button" class="cmd-btn cmd-btn-secondary cmd-btn-sm" @click="close">Cancel</button>
            <button
              type="button"
              class="cmd-btn cmd-btn-sm"
              :class="`cmd-btn-${confirmVariant}`"
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

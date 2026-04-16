<template>
  <Teleport to="body">
    <div v-if="isOpen" class="cmd-modal-backdrop" @click.self="close">
      <div class="cmd-modal-dialog">
        <div class="cmd-modal-content">
          <div class="cmd-modal-header">
            <h5 class="cmd-modal-title">Edit Stow Target</h5>
            <button type="button" class="cmd-btn-close" @click="close"><i class="bi bi-x-lg"></i></button>
          </div>
          <div class="cmd-modal-body">
            <div class="mb-4">
              <button
                type="button"
                class="cmd-btn cmd-btn-sm cmd-btn-outline-primary w-full"
                :disabled="capturing"
                @click="capturePose"
              >
                {{ capturing ? 'Capturing...' : 'Capture current arm pose' }}
              </button>
              <small v-if="captureError" class="text-danger block mt-1">{{ captureError }}</small>
            </div>
            <div class="grid grid-cols-3 gap-3 mb-3">
              <div>
                <label class="cmd-form-label">X (m):</label>
                <input class="cmd-form-control cmd-form-control-sm" v-model.number="editData.x" type="number" step="0.001" />
              </div>
              <div>
                <label class="cmd-form-label">Y (m):</label>
                <input class="cmd-form-control cmd-form-control-sm" v-model.number="editData.y" type="number" step="0.001" />
              </div>
              <div>
                <label class="cmd-form-label">Z (m):</label>
                <input class="cmd-form-control cmd-form-control-sm" v-model.number="editData.z" type="number" step="0.001" />
              </div>
            </div>
            <div class="grid grid-cols-2 gap-3">
              <div>
                <label class="cmd-form-label">Pitch (rad):</label>
                <input class="cmd-form-control cmd-form-control-sm" v-model.number="editData.pitch" type="number" step="0.001" />
              </div>
              <div>
                <label class="cmd-form-label">Roll (rad):</label>
                <input class="cmd-form-control cmd-form-control-sm" v-model.number="editData.roll" type="number" step="0.001" />
              </div>
            </div>
            <small v-if="saveError" class="text-danger block mt-2">{{ saveError }}</small>
          </div>
          <div class="cmd-modal-footer">
            <button type="button" class="cmd-btn cmd-btn-sm cmd-btn-danger me-auto" :disabled="resetting" @click="resetToDefaults">
              {{ resetting ? 'Resetting...' : 'Reset to defaults' }}
            </button>
            <button type="button" class="cmd-btn cmd-btn-sm cmd-btn-secondary" @click="close">Cancel</button>
            <button type="button" class="cmd-btn cmd-btn-sm cmd-btn-primary" :disabled="saving" @click="save">
              {{ saving ? 'Saving...' : 'Save' }}
            </button>
          </div>
        </div>
      </div>
    </div>
  </Teleport>
</template>

<script lang="ts" setup>
import { ref, watch } from 'vue'
import { armAPI } from '@/utils/api'
import type { StowPosition } from '@/utils/apiTypes'

const props = defineProps<{
  isOpen: boolean
  initial: StowPosition
}>()

const emit = defineEmits<{
  close: []
  saved: [value: StowPosition]
}>()

const editData = ref<StowPosition>({ ...props.initial })
const capturing = ref(false)
const saving = ref(false)
const resetting = ref(false)
const captureError = ref('')
const saveError = ref('')

watch(
  () => props.isOpen,
  (open) => {
    if (open) {
      editData.value = { ...props.initial }
      captureError.value = ''
      saveError.value = ''
    }
  },
)

function close() {
  emit('close')
}

async function capturePose() {
  capturing.value = true
  captureError.value = ''
  try {
    const result = await armAPI.captureStowPose()
    if (result.status === 'success' && result.stow_position) {
      editData.value = { ...result.stow_position }
    } else {
      captureError.value = result.message ?? 'Failed to capture arm pose.'
    }
  } catch (err) {
    captureError.value = err instanceof Error ? err.message : 'Failed to capture arm pose.'
  } finally {
    capturing.value = false
  }
}

async function resetToDefaults() {
  resetting.value = true
  saveError.value = ''
  try {
    const result = await armAPI.resetStowConfig()
    if (result.status === 'success' && result.stow_position) {
      editData.value = { ...result.stow_position }
      emit('saved', result.stow_position)
    }
  } catch (err) {
    saveError.value = err instanceof Error ? err.message : 'Failed to reset stow position.'
  } finally {
    resetting.value = false
  }
}

async function save() {
  saving.value = true
  saveError.value = ''
  try {
    const result = await armAPI.saveStowConfig({ ...editData.value })
    if (result.status === 'success' && result.stow_position) {
      emit('saved', result.stow_position)
      emit('close')
    } else {
      saveError.value = result.message ?? 'Failed to save stow config.'
    }
  } catch (err) {
    saveError.value = err instanceof Error ? err.message : 'Failed to save stow config.'
  } finally {
    saving.value = false
  }
}
</script>

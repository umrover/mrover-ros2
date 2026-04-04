<template>
  <div
    class="list-item"
    :class="{ 'kbd-highlighted': highlighted, 'kbd-visual-selected': visualSelected }"
    data-testid="pw-waypoint-store-item"
  >
    <template v-if="editing">
      <div class="flex flex-col gap-2">
        <input
          ref="nameInputRef"
          class="form-control form-control-sm"
          v-model="editData.name"
          placeholder="Name"
          @keydown="handleEditKeydown"
        />
        <div class="grid grid-cols-2 gap-2">
          <input
            ref="latInputRef"
            class="form-control form-control-sm"
            v-model.number="editData.lat"
            type="number"
            step="0.000001"
            placeholder="Latitude"
            @keydown="handleEditKeydown"
          />
          <input
            ref="lonInputRef"
            class="form-control form-control-sm"
            v-model.number="editData.lon"
            type="number"
            step="0.000001"
            placeholder="Longitude"
            @keydown="handleEditKeydown"
          />
        </div>
        <div class="flex justify-end gap-1">
          <small class="text-muted self-center mr-auto">Tab: next field, Enter: save, Esc: cancel</small>
        </div>
      </div>
    </template>

    <template v-else>
      <div class="flex justify-between items-center mb-1">
        <div class="flex items-center gap-2">
          <i class="bi bi-grip-vertical drag-handle"></i>
          <h5 class="list-item-title" data-testid="pw-waypoint-name">{{ waypoint.name }}</h5>
        </div>
        <div v-if="waypoint.tag_id != null" class="flex items-center gap-2">
          <span class="data-label">Tag ID: {{ waypoint.tag_id }}</span>
        </div>
      </div>
      <div class="flex justify-between items-center">
        <small class="text-muted">{{ waypoint.lat.toFixed(6) }}N, {{ waypoint.lon.toFixed(6) }}W</small>
        <div class="flex gap-1">
          <button class="btn btn-success btn-sm btn-icon" @click="emit('add', waypoint)"><i class="bi bi-plus-lg" /></button>
          <button class="btn btn-warning btn-sm btn-icon" @click="openEditModal"><i class="bi bi-pencil-fill" /></button>
          <button
            class="btn btn-danger btn-sm btn-icon"
            :disabled="waypoint.deletable === false"
            @click="emit('delete', index)"
          ><i class="bi bi-trash-fill" /></button>
        </div>
      </div>
    </template>

    <Teleport to="body">
      <div v-if="isOpen" class="modal-backdrop" @click.self="closeEditModal">
        <div class="modal-dialog">
          <div class="modal-content">
            <div class="modal-header">
              <h5 class="modal-title">Edit Waypoint</h5>
              <button type="button" class="btn-close" @click="closeEditModal"><i class="bi bi-x-lg" /></button>
            </div>
            <div class="modal-body">
              <div class="mb-4">
                <label class="form-label">Name:</label>
                <input class="form-control" v-model="modalData.name" />
              </div>
              <div class="grid grid-cols-2 gap-4 mb-4">
                <div>
                  <label class="form-label">Latitude:</label>
                  <input class="form-control" v-model.number="modalData.lat" type="number" step="0.000001" />
                </div>
                <div>
                  <label class="form-label">Longitude:</label>
                  <input class="form-control" v-model.number="modalData.lon" type="number" step="0.000001" />
                </div>
              </div>
            </div>
            <div class="modal-footer">
              <button type="button" class="btn btn-secondary" @click="closeEditModal">Cancel</button>
              <button type="button" class="btn btn-primary" @click="saveModalEdit">Save</button>
            </div>
          </div>
        </div>
      </div>
    </Teleport>
  </div>
</template>

<script lang="ts" setup>
import { ref, watch, nextTick } from 'vue'
import type { AutonWaypoint } from '@/types/waypoints'
import { useModal } from '@/composables/useModal'

const props = defineProps<{
  waypoint: AutonWaypoint
  index: number
  highlighted?: boolean
  visualSelected?: boolean
  editing?: boolean
}>()

const emit = defineEmits<{
  add: [waypoint: AutonWaypoint]
  delete: [index: number]
  update: [waypoint: AutonWaypoint, index: number]
  'save-edit': [waypoint: AutonWaypoint, index: number]
  'cancel-edit': []
}>()

const editData = ref({ name: '', lat: 0, lon: 0 })
const nameInputRef = ref<HTMLInputElement | null>(null)
const latInputRef = ref<HTMLInputElement | null>(null)
const lonInputRef = ref<HTMLInputElement | null>(null)

watch(() => props.editing, (isEditing) => {
  if (isEditing) {
    editData.value = {
      name: props.waypoint.name,
      lat: props.waypoint.lat,
      lon: props.waypoint.lon,
    }
    nextTick(() => {
      nameInputRef.value?.focus()
      nameInputRef.value?.select()
    })
  }
})

function getEditInputs(): HTMLInputElement[] {
  return [nameInputRef.value, latInputRef.value, lonInputRef.value].filter(
    (el): el is HTMLInputElement => el != null
  )
}

function handleEditKeydown(e: KeyboardEvent) {
  if (e.key === 'Enter') {
    e.preventDefault()
    emit('save-edit', {
      ...props.waypoint,
      name: editData.value.name,
      lat: editData.value.lat,
      lon: editData.value.lon,
    }, props.index)
    return
  }

  if (e.key === 'Escape') {
    e.preventDefault()
    emit('cancel-edit')
    return
  }

  if (e.key === 'Tab') {
    e.preventDefault()
    const inputs = getEditInputs()
    const current = inputs.indexOf(e.target as HTMLInputElement)
    if (current === -1) return
    const direction = e.shiftKey ? -1 : 1
    const nextIndex = (current + direction + inputs.length) % inputs.length
    inputs[nextIndex]?.focus()
    inputs[nextIndex]?.select()
  }
}

const { isOpen, show, hide } = useModal()
const modalData = ref({ name: '', lat: 0, lon: 0 })

function openEditModal() {
  modalData.value = {
    name: props.waypoint.name,
    lat: props.waypoint.lat,
    lon: props.waypoint.lon,
  }
  show()
}

function closeEditModal() {
  hide()
}

function saveModalEdit() {
  emit('update', {
    ...props.waypoint,
    name: modalData.value.name,
    lat: modalData.value.lat,
    lon: modalData.value.lon,
  }, props.index)
  closeEditModal()
}
</script>

<style scoped>
.list-item-title {
  font-size: 0.875rem;
  font-weight: 600;
}

.data-label {
  font-size: 0.6875rem;
  color: var(--text-muted);
}

.btn-icon {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  width: 1.75rem;
  height: 1.75rem;
  padding: 0;
  font-size: 0.75rem;
}

.drag-handle {
  cursor: grab;
  opacity: 0.4;
}

.drag-handle:hover {
  opacity: 1;
}
</style>

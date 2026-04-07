<template>
  <div
    class="list-item"
    :class="{ 'kbd-highlighted': highlighted, 'kbd-visual-selected': visualSelected }"
    data-testid="pw-waypoint-store-item"
  >
    <div class="flex justify-between items-center mb-1">
        <div class="flex items-center gap-2">
          <i class="bi bi-grip-vertical drag-handle"></i>
          <h5 class="list-item-title" data-testid="pw-waypoint-name">{{ waypoint.name }}</h5>
        </div>
        <div class="flex items-center gap-2">
          <span v-if="waypoint.type === 1 && waypoint.tag_id != null" class="data-label">Tag ID: {{ waypoint.tag_id }}</span>
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

    <Teleport to="body">
      <div v-if="isOpen" class="modal-backdrop" tabindex="-1" @click.self="closeEditModal" @keydown="handleModalKeydown" @keydown.tab.prevent="trapFocus">
        <div class="modal-dialog">
          <div ref="modalContentRef" class="modal-content">
            <div class="modal-header">
              <h5 class="modal-title">Edit Waypoint</h5>
              <button type="button" class="btn-close" tabindex="-1" @click="closeEditModal"><i class="bi bi-x-lg" /></button>
            </div>
            <div class="modal-body">
              <div class="mb-4">
                <label class="form-label">Name:</label>
                <input ref="modalNameRef" class="form-control" v-model="modalData.name" :disabled="waypoint.deletable === false" />
              </div>
              <div class="grid grid-cols-2 gap-4 mb-4">
                <div>
                  <label class="form-label">Type:</label>
                  <select class="form-select" v-model.number="modalData.type" :disabled="waypoint.deletable === false">
                    <option :value="0">No Search</option>
                    <option :value="1">Post</option>
                    <option :value="2">Mallet</option>
                    <option :value="3">Water Bottle</option>
                    <option :value="4">Rock Pick</option>
                  </select>
                </div>
                <div>
                  <label class="form-label">Tag ID:</label>
                  <input
                    v-if="modalData.type === 1"
                    class="form-control"
                    v-model.number="modalData.tag_id"
                    type="number"
                    min="0"
                    max="249"
                    step="1"
                  />
                  <input v-else class="form-control" type="number" placeholder="N/A" disabled />
                </div>
              </div>
              <div class="grid grid-cols-2 gap-4 mb-4">
                <div>
                  <label class="form-label">Latitude:</label>
                  <input ref="modalLatRef" class="form-control" v-model.number="modalData.lat" type="number" step="0.000001" />
                </div>
                <div>
                  <label class="form-label">Longitude:</label>
                  <input class="form-control" v-model.number="modalData.lon" type="number" step="0.000001" />
                </div>
              </div>
            </div>
            <div class="modal-footer">
              <button type="button" class="btn btn-danger" tabindex="-1" @click="closeEditModal">Cancel</button>
              <button type="button" class="btn btn-success" tabindex="-1" @click="saveAndClose">Save</button>
            </div>
          </div>
        </div>
      </div>
    </Teleport>
  </div>
</template>

<script lang="ts" setup>
import { ref, computed, nextTick } from 'vue'
import type { AutonWaypoint } from '@/types/waypoints'
import { useModal } from '@/composables/useModal'

const props = defineProps<{
  waypoint: AutonWaypoint
  index: number
  highlighted?: boolean
  visualSelected?: boolean
  onColon?: () => void
}>()

const TYPE_LABELS: Record<number, string> = {
  0: 'No Search',
  1: 'Post',
  2: 'Mallet',
  3: 'Water Bottle',
  4: 'Rock Pick',
}

const typeLabel = computed(() => TYPE_LABELS[props.waypoint.type] ?? `Type ${props.waypoint.type}`)

const emit = defineEmits<{
  add: [waypoint: AutonWaypoint]
  delete: [index: number]
  update: [waypoint: AutonWaypoint, index: number]
}>()

const { isOpen, show, hide } = useModal()
const modalData = ref({ name: '', type: 0, tag_id: null as number | null, lat: 0, lon: 0 })
const modalContentRef = ref<HTMLElement | null>(null)
const modalNameRef = ref<HTMLInputElement | null>(null)
const modalLatRef = ref<HTMLInputElement | null>(null)

function trapFocus(e: KeyboardEvent) {
  const container = modalContentRef.value
  if (!container) return
  const focusable = Array.from(
    container.querySelectorAll<HTMLElement>('input:not(:disabled), select:not(:disabled)')
  )
  if (focusable.length === 0) return
  const current = focusable.indexOf(document.activeElement as HTMLElement)
  const direction = e.shiftKey ? -1 : 1
  const next = (current + direction + focusable.length) % focusable.length
  focusable[next]?.focus()
}

function handleModalKeydown(e: KeyboardEvent) {
  if (e.key === ':') {
    e.preventDefault()
    if (props.onColon) props.onColon()
    return
  }
  if (e.key === 'Escape') {
    e.preventDefault()
    closeEditModal()
    return
  }
  if (e.key === 'Enter') {
    e.preventDefault()
    saveAndClose()
    return
  }
}

function openEditModal() {
  modalData.value = {
    name: props.waypoint.name,
    type: props.waypoint.type,
    tag_id: props.waypoint.tag_id,
    lat: props.waypoint.lat,
    lon: props.waypoint.lon,
  }
  show()
  nextTick(() => {
    const target = props.waypoint.deletable === false ? modalLatRef.value : modalNameRef.value
    target?.focus()
    target?.select()
  })
}

function saveEdit() {
  emit('update', {
    ...props.waypoint,
    name: modalData.value.name,
    type: modalData.value.type,
    tag_id: modalData.value.type === 1 ? modalData.value.tag_id : null,
    lat: modalData.value.lat,
    lon: modalData.value.lon,
  }, props.index)
}

function closeEditModal() {
  hide()
}

function saveAndClose() {
  saveEdit()
  closeEditModal()
}

defineExpose({ openEditModal, closeEditModal, saveEdit, saveAndClose, isOpen })
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

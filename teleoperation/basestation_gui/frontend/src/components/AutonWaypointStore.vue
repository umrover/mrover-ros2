<template>
  <div class="list-item" data-testid="pw-waypoint-store-item">
    <div class="flex justify-between items-center mb-1">
      <h5 class="list-item-title" data-testid="pw-waypoint-name">{{ waypoint.name }}</h5>
      <div v-if="waypoint.tag_id != null" class="flex items-center gap-2">
        <span class="data-label">Tag ID: {{ waypoint.tag_id }}</span>
      </div>
    </div>
    <div class="flex justify-between items-center">
      <small class="text-muted">{{ waypoint.lat.toFixed(6) }}N, {{ waypoint.lon.toFixed(6) }}W</small>
      <div class="flex gap-1">
        <button class="btn btn-success btn-sm btn-icon" @click="emit('add', waypoint)"><i class="bi bi-plus-lg"></i></button>
        <button class="btn btn-warning btn-sm btn-icon" @click="openEditModal"><i class="bi bi-pencil-fill"></i></button>
        <button
          class="btn btn-danger btn-sm btn-icon"
          :disabled="waypoint.deletable === false"
          @click="emit('delete', index)"
        ><i class="bi bi-trash-fill"></i></button>
      </div>
    </div>

    <Teleport to="body">
      <div v-if="isOpen" class="modal-backdrop" @click.self="closeEditModal">
        <div class="modal-dialog">
          <div class="modal-content">
            <div class="modal-header">
              <h5 class="modal-title">Edit Waypoint</h5>
              <button type="button" class="btn-close" @click="closeEditModal"><i class="bi bi-x-lg"></i></button>
            </div>
            <div class="modal-body">
              <div class="mb-4">
                <label class="form-label">Name:</label>
                <input class="form-control" v-model="editData.name" />
              </div>
              <div class="grid grid-cols-2 gap-4 mb-4">
                <div>
                  <label class="form-label">Latitude:</label>
                  <input class="form-control" v-model.number="editData.lat" type="number" step="0.000001" />
                </div>
                <div>
                  <label class="form-label">Longitude:</label>
                  <input class="form-control" v-model.number="editData.lon" type="number" step="0.000001" />
                </div>
              </div>
            </div>
            <div class="modal-footer">
              <button type="button" class="btn btn-secondary" @click="closeEditModal">Cancel</button>
              <button type="button" class="btn btn-primary" @click="saveEdit">Save</button>
            </div>
          </div>
        </div>
      </div>
    </Teleport>
  </div>
</template>

<script lang="ts" setup>
import { ref } from 'vue'
import type { AutonWaypoint } from '@/types/waypoints'
import { useModal } from '@/composables/useModal'

const props = defineProps<{
  waypoint: AutonWaypoint
  index: number
}>()

const emit = defineEmits<{
  add: [waypoint: AutonWaypoint]
  delete: [index: number]
  update: [waypoint: AutonWaypoint, index: number]
}>()

const { isOpen, show, hide } = useModal()
const editData = ref({
  name: '',
  lat: 0,
  lon: 0,
})

function openEditModal() {
  editData.value = {
    name: props.waypoint.name,
    lat: props.waypoint.lat,
    lon: props.waypoint.lon,
  }
  show()
}

function closeEditModal() {
  hide()
}

function saveEdit() {
  emit('update', {
    ...props.waypoint,
    name: editData.value.name,
    lat: editData.value.lat,
    lon: editData.value.lon,
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
</style>

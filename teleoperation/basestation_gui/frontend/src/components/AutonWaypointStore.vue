<template>
  <div class="cmd-list-item" data-testid="pw-waypoint-store-item">
    <div class="flex justify-between items-center mb-1">
      <h5 class="cmd-list-item-title" data-testid="pw-waypoint-name">{{ waypoint.name }}</h5>
      <div class="flex items-center gap-2">
        <span class="cmd-data-label">Tag ID: {{ waypoint.tag_id }}</span>
      </div>
    </div>
    <div class="flex justify-between items-center">
      <small class="text-muted">{{ waypoint.lat.toFixed(6) }}N, {{ waypoint.lon.toFixed(6) }}W</small>
      <div class="flex gap-1">
        <button class="cmd-btn cmd-btn-success cmd-btn-sm cmd-btn-icon" @click="emit('add', waypoint)"><i class="bi bi-plus-lg"></i></button>
        <button class="cmd-btn cmd-btn-warning cmd-btn-sm cmd-btn-icon" @click="openEditModal"><i class="bi bi-pencil-fill"></i></button>
        <button
          class="cmd-btn cmd-btn-danger cmd-btn-sm cmd-btn-icon"
          :disabled="waypoint.deletable === false"
          @click="emit('delete', index)"
        ><i class="bi bi-trash-fill"></i></button>
      </div>
    </div>

    <Teleport to="body">
      <div v-if="isOpen" class="cmd-modal-backdrop" @click.self="closeEditModal">
        <div class="cmd-modal-dialog">
          <div class="cmd-modal-content">
            <div class="cmd-modal-header">
              <h5 class="cmd-modal-title">Edit Waypoint</h5>
              <button type="button" class="cmd-btn-close" @click="closeEditModal"><i class="bi bi-x-lg"></i></button>
            </div>
            <div class="cmd-modal-body">
              <div class="mb-4">
                <label class="cmd-form-label">Name:</label>
                <input class="cmd-form-control" v-model="editData.name" />
              </div>
              <div class="grid grid-cols-2 gap-4 mb-4">
                <div>
                  <label class="cmd-form-label">Latitude:</label>
                  <input class="cmd-form-control" v-model.number="editData.lat" type="number" step="0.000001" />
                </div>
                <div>
                  <label class="cmd-form-label">Longitude:</label>
                  <input class="cmd-form-control" v-model.number="editData.lon" type="number" step="0.000001" />
                </div>
              </div>
            </div>
            <div class="cmd-modal-footer">
              <button type="button" class="cmd-btn cmd-btn-secondary" @click="closeEditModal">Cancel</button>
              <button type="button" class="cmd-btn cmd-btn-primary" @click="saveEdit">Save</button>
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
.cmd-list-item-title {
  font-size: 0.875rem;
  font-weight: 600;
}

.cmd-data-label {
  font-size: 0.6875rem;
  color: var(--text-muted);
}

.cmd-btn-icon {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  width: 1.75rem;
  height: 1.75rem;
  padding: 0;
  font-size: 0.75rem;
}
</style>

<template>
  <div class="cmd-list-item p-2 rounded border border-2 mb-1" data-testid="pw-waypoint-store-item">
    <div class="d-flex justify-content-between align-items-center mb-1">
      <h5 class="cmd-list-item-title m-0" data-testid="pw-waypoint-name">{{ waypoint.name }}</h5>
      <div class="d-flex align-items-center gap-2">
        <span class="cmd-data-label">ID: {{ waypoint.id }}</span>
        <span class="cmd-data-label">R: {{ waypoint.coverage_radius }}</span>
      </div>
    </div>
    <div class="d-flex justify-content-between align-items-center">
      <small class="text-muted">{{ waypoint.lat.toFixed(6) }}N, {{ waypoint.lon.toFixed(6) }}W</small>
      <div class="d-flex gap-1">
        <button class="btn btn-success btn-sm border-2 cmd-btn-icon" @click="$emit('add', waypoint)"><i class="bi bi-plus-lg"></i></button>
        <button class="btn btn-warning btn-sm border-2 cmd-btn-icon" @click="openEditModal"><i class="bi bi-pencil-fill"></i></button>
        <button
          class="btn btn-danger btn-sm border-2 cmd-btn-icon"
          :disabled="waypoint.deletable === false"
          @click="$emit('delete', index)"
        ><i class="bi bi-trash-fill"></i></button>
      </div>
    </div>

    <Teleport to="body">
      <div class="modal fade" :id="'editModal-' + index" tabindex="-1" role="dialog">
        <div class="modal-dialog modal-dialog-centered" role="document">
          <div class="modal-content">
            <div class="modal-header">
              <h5 class="modal-title">Edit Waypoint</h5>
              <button type="button" class="btn-close" @click="closeEditModal"></button>
            </div>
            <div class="modal-body">
              <div class="mb-3">
                <label class="form-label">Name:</label>
                <input class="form-control" v-model="editData.name" />
              </div>
              <div class="row mb-3">
                <div class="col-6">
                  <label class="form-label">Latitude:</label>
                  <input class="form-control" v-model.number="editData.lat" type="number" step="0.000001" />
                </div>
                <div class="col-6">
                  <label class="form-label">Longitude:</label>
                  <input class="form-control" v-model.number="editData.lon" type="number" step="0.000001" />
                </div>
              </div>
              <div class="mb-3">
                <label class="form-label">Coverage Radius (0 for default):</label>
                <input
                  class="form-control"
                  v-model.number="editData.coverage_radius"
                  type="number"
                  step="0.5"
                  min="0"
                />
              </div>
            </div>
            <div class="modal-footer">
              <button type="button" class="btn btn-secondary border-2" @click="closeEditModal">Cancel</button>
              <button type="button" class="btn btn-primary border-2" @click="saveEdit">Save</button>
            </div>
          </div>
        </div>
      </div>
    </Teleport>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import type { PropType } from 'vue'
import type { AutonWaypoint } from '@/types/waypoints'
import { Modal } from 'bootstrap'

export default defineComponent({
  name: 'WaypointStore',
  props: {
    waypoint: {
      type: Object as PropType<AutonWaypoint>,
      required: true,
    },
    index: {
      type: Number,
      required: true,
    },
  },
  emits: ['add', 'delete', 'update'],
  data() {
    return {
      editModal: null as Modal | null,
      editData: {
        name: '',
        lat: 0,
        lon: 0,
        coverage_radius: 0,
      },
    }
  },
  methods: {
    openEditModal() {
      this.editData = {
        name: this.waypoint.name,
        lat: this.waypoint.lat,
        lon: this.waypoint.lon,
        coverage_radius: this.waypoint.coverage_radius,
      }
      if (!this.editModal) {
        this.editModal = new Modal(`#editModal-${this.index}`, {})
      }
      this.editModal.show()
    },
    closeEditModal() {
      this.editModal?.hide()
    },
    saveEdit() {
      const updatedWaypoint = {
        ...this.waypoint,
        name: this.editData.name,
        lat: this.editData.lat,
        lon: this.editData.lon,
        coverage_radius: this.editData.coverage_radius,
      }
      this.$emit('update', updatedWaypoint, this.index)
      this.closeEditModal()
    },
  },
})
</script>

<style scoped>
.cmd-list-item {
  background-color: var(--card-bg);
}

.cmd-list-item-title {
  font-size: 0.875rem;
  font-weight: 600;
}

.cmd-data-label {
  font-size: 0.6875rem;
  color: var(--text-muted);
}

.cmd-btn-icon {
  width: 1.75rem;
  height: 1.75rem;
  padding: 0;
  display: inline-flex;
  align-items: center;
  justify-content: center;
  font-size: 0.75rem;
}
</style>

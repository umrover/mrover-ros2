<template>
  <div class="waypoints p-2 mb-2">
    <div class="waypoint-header mb-1">
      <h5 class="mb-0">{{ waypoint.name }}</h5>
      <small class="text-muted">TAG ID: {{ waypoint.tag_id }}</small>
    </div>
    <div>
      <div class="input-group mb-1">
        <input
          class="form-control"
          v-model.number="localLat" :id="'lat-' + waypoint.tag_id"
        />
        <span class="input-group-text">ºN</span>
      </div>
      <div class="input-group mb-1">
        <input
          class="form-control"
          v-model.number="localLon" :id="'lon-' + waypoint.tag_id"
        />
        <span class="input-group-text">ºW</span>
      </div>
      <div class="waypoint-button-row">
        <button class="btn btn-success" @click="addWaypoint">Add</button>
        <button
          class="btn btn-danger"
          :disabled="waypoint.deletable === false"
          @click="$emit('delete', index)"
        >
          Delete
        </button>
      </div>
    </div>
    <div class="mb-2">
      <small class="text-muted">{{ waypoint.lat.toFixed(6) }}N, {{ waypoint.lon.toFixed(6) }}W</small>
    </div>
    <div class="d-flex gap-1">
      <button class="btn btn-success btn-sm border-2 cmd-btn-text" @click="$emit('add', waypoint)">Add</button>
      <button class="btn btn-warning btn-sm border-2 cmd-btn-text" @click="openEditModal">Edit</button>
      <button
        class="btn btn-danger btn-sm border-2 cmd-btn-text"
        :disabled="waypoint.deletable === false"
        @click="$emit('delete', index)"
      >
        Delete
      </button>
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
      type: Object as PropType<Waypoint>,
      required: true,
    },
    index: {
      type: Number,
      required: true,
    },
  },
  emits: ['add', 'delete'],
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

.cmd-btn-text {
  font-size: 0.6875rem;
  font-weight: 500;
  text-transform: uppercase;
  letter-spacing: 0.03em;
  flex: 1 1 0;
}
</style>

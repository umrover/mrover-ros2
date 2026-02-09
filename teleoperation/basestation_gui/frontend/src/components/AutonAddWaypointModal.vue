<template>
  <Teleport to="body">
    <div class="modal fade" id="modalWypt" tabindex="-1" role="dialog" data-testid="pw-waypoint-modal">
      <div class="modal-dialog modal-dialog-centered" role="document">
        <div class="modal-content">
          <div class="modal-header">
            <h5 class="modal-title">Add Waypoint</h5>
            <button type="button" class="btn-close" @click="close"></button>
          </div>
          <div class="modal-body">
            <div class="row g-3">
              <div class="col-md-6">
                <label for="waypointname" class="form-label">Name:</label>
                <input
                  class="form-control"
                  id="waypointname"
                  data-testid="pw-waypoint-name-input"
                  v-model="form.name"
                />
              </div>
              <div class="col-md-6">
                <label for="waypointid" class="form-label">Tag ID:</label>
                <input
                  v-if="form.type == 1"
                  class="form-control"
                  id="waypointid"
                  v-model="form.tag_id"
                  type="number"
                  max="249"
                  min="0"
                  step="1"
                />
                <input
                  v-else
                  class="form-control"
                  id="waypointid"
                  type="number"
                  placeholder="-1"
                  disabled
                />
              </div>
              <div class="col-12">
                <label class="form-label">Type:</label>
                <select class="form-select" v-model="form.type">
                  <option value="0">No Search</option>
                  <option value="1">Post</option>
                  <option value="2">Mallet</option>
                  <option value="3">Water Bottle</option>
                  <option value="4">Rock Pick</option>
                </select>
              </div>
              <div class="col-12">
                <label for="coverage_radius" class="form-label">Coverage Radius (0 for default):</label>
                <input
                  class="form-control"
                  id="coverage_radius"
                  v-model.number="form.coverage_radius"
                  type="number"
                  step="0.5"
                  min="0"
                />
              </div>
            </div>
          </div>
          <div class="modal-footer">
            <button
              type="button"
              class="btn btn-secondary border-2"
              data-testid="pw-add-waypoint-submit"
              @click="submit"
            >
              Add Waypoint
            </button>
          </div>
        </div>
      </div>
    </div>
  </Teleport>
</template>

<script lang="ts" setup>
import { ref, onMounted } from 'vue'
import { Modal } from 'bootstrap'
import { useAutonomyStore } from '@/stores/autonomy'

const autonomyStore = useAutonomyStore()
const modal = ref<Modal | null>(null)

const defaultForm = () => ({
  name: '',
  tag_id: -1,
  type: 0,
  coverage_radius: 0,
})

const form = ref(defaultForm())

onMounted(() => {
  modal.value = new Modal('#modalWypt', {})
})

function open() {
  modal.value?.show()
}

function close() {
  if (document.activeElement instanceof HTMLElement) {
    document.activeElement.blur()
  }
  modal.value?.hide()
}

async function submit() {
  const payload = {
    ...form.value,
    lat: autonomyStore.clickPoint.lat,
    lon: autonomyStore.clickPoint.lon,
    enable_costmap: true,
  }

  try {
    await autonomyStore.createWaypoint(payload)
  } catch (error) {
    console.error('Failed to create waypoint:', error)
    return
  }

  form.value = defaultForm()
  close()
}

defineExpose({ open, close })
</script>

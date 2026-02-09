<template>
  <Teleport to="body">
    <div v-if="isOpen" class="cmd-modal-backdrop" @click.self="close" data-testid="pw-waypoint-modal">
      <div class="cmd-modal-dialog">
        <div class="cmd-modal-content">
          <div class="cmd-modal-header">
            <h5 class="cmd-modal-title">Add Waypoint</h5>
            <button type="button" class="cmd-btn-close" @click="close"><i class="bi bi-x-lg"></i></button>
          </div>
          <div class="cmd-modal-body">
            <div class="grid grid-cols-2 gap-4">
              <div>
                <label for="waypointname" class="cmd-form-label">Name:</label>
                <input
                  class="cmd-form-control"
                  id="waypointname"
                  data-testid="pw-waypoint-name-input"
                  v-model="form.name"
                />
              </div>
              <div>
                <label for="waypointid" class="cmd-form-label">Tag ID:</label>
                <input
                  v-if="form.type == 1"
                  class="cmd-form-control"
                  id="waypointid"
                  v-model="form.tag_id"
                  type="number"
                  max="249"
                  min="0"
                  step="1"
                />
                <input
                  v-else
                  class="cmd-form-control"
                  id="waypointid"
                  type="number"
                  placeholder="-1"
                  disabled
                />
              </div>
              <div class="col-span-2">
                <label class="cmd-form-label">Type:</label>
                <select class="cmd-form-select" v-model="form.type">
                  <option value="0">No Search</option>
                  <option value="1">Post</option>
                  <option value="2">Mallet</option>
                  <option value="3">Water Bottle</option>
                  <option value="4">Rock Pick</option>
                </select>
              </div>
              <div class="col-span-2">
                <label for="coverage_radius" class="cmd-form-label">Coverage Radius (0 for default):</label>
                <input
                  class="cmd-form-control"
                  id="coverage_radius"
                  v-model.number="form.coverage_radius"
                  type="number"
                  step="0.5"
                  min="0"
                />
              </div>
            </div>
          </div>
          <div class="cmd-modal-footer">
            <button
              type="button"
              class="cmd-btn cmd-btn-secondary"
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
import { ref } from 'vue'
import { useModal } from '@/composables/useModal'
import { useAutonomyStore } from '@/stores/autonomy'

const autonomyStore = useAutonomyStore()
const { isOpen, show, hide } = useModal()

const defaultForm = () => ({
  name: '',
  tag_id: -1,
  type: 0,
  coverage_radius: 0,
})

const form = ref(defaultForm())

function open() { show() }

function close() {
  if (document.activeElement instanceof HTMLElement) {
    document.activeElement.blur()
  }
  hide()
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

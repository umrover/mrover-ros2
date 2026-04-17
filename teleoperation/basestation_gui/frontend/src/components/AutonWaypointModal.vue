<template>
  <Teleport to="body">
    <div v-if="isOpen" class="modal-backdrop" @click.self="close" @keydown.escape="close" @keydown.enter.prevent="submit" data-testid="pw-waypoint-modal">
      <div class="modal-dialog">
        <div class="modal-content">
          <div class="modal-header">
            <h5 class="modal-title">Add Waypoint</h5>
            <button type="button" class="btn-close" @click="close"><i class="bi bi-x-lg"></i></button>
          </div>
          <div class="modal-body">
            <div class="grid grid-cols-2 gap-4">
              <div>
                <label for="waypointname" class="form-label">Name:</label>
                <input
                  class="form-control"
                  id="waypointname"
                  data-testid="pw-waypoint-name-input"
                  v-model="form.name"
                />
              </div>
              <div>
                <label for="waypointid" class="form-label">Tag ID:</label>
                <input
                  v-if="form.type === 1"
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
              <div class="col-span-2">
                <label class="form-label">Type:</label>
                <select class="form-select" v-model.number="form.type">
                  <option :value="0">No Search</option>
                  <option :value="1">Post</option>
                  <option :value="2">Mallet</option>
                  <option :value="3">Water Bottle</option>
                  <option :value="4">Rock Pick</option>
                </select>
              </div>
            </div>
          </div>
          <div class="modal-footer">
            <button
              type="button"
              class="btn btn-secondary"
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
  tag_id: null as number | null,
  type: 0,
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
    tag_id: form.value.type === 1 ? form.value.tag_id : null,
    lat: autonomyStore.clickPoint.lat,
    lon: autonomyStore.clickPoint.lon,
    enable_costmap: true,
  }

  try {
    await autonomyStore.addToStore(payload)
  } catch (error) {
    console.error('Failed to create waypoint:', error)
    return
  }

  form.value = defaultForm()
  close()
}

defineExpose({ open, close })
</script>

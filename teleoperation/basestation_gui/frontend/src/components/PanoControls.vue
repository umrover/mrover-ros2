<template>
  <div class="d-flex justify-content-between p-2 gap-2">
    <h4 class="m-0 font-monospace align-content-center">Panorama</h4>
    <FeedbackButton
      name="Start Panorama"
      mode="action"
      active-text="Capturing..."
      complete-text="Complete"
    />
  </div>
</template>

<script lang="ts">
import FeedbackButton from './FeedbackButton.vue'

export default {
  components: { FeedbackButton },
}
</script>

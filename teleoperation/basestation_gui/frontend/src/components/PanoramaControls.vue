<template>
  <div class="d-flex flex-column gap-2 p-2">
    <div class="d-flex justify-content-between align-items-center">
      <h4 class="m-0">Panorama Controls</h4>
    </div>
    <FeedbackButton
      name="Start Panorama"
      mode="action"
      active-text="Capturing..."
      complete-text="Complete"
    />
  </div>
</template>

<script lang="ts" setup>
import FeedbackButton from './FeedbackButton.vue'
</script>

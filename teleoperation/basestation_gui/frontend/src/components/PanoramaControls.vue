<template>
  <div class="d-flex flex-row gap-2 justify-content-between align-items-center">
    <h4 class="m-0">Panorama</h4>
    <FeedbackButton
      name="Start"
      mode="action"
      active-text="Capturing..."
      complete-text="Complete"
    />
  </div>
</template>

<script lang="ts" setup>
import FeedbackButton from './FeedbackButton.vue'
</script>

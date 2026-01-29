<template>
  <div class="d-flex flex-column gap-2 w-100 h-100">
    <h4 class="component-header">Panorama</h4>
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


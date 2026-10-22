<template>
  <!-- <div class="flex-1 flex"> -->
  <h1>Astronaut Status</h1>
  <!-- </div> -->
  <div class="grid grid-cols-4 grid-rows-3">
    <!-- <div class="border-2 rounded px-1 ml-1 flex flex-col justify-center"> -->
      <div class="flex gap-2 long-bar"> <span class="text-muted">Video</span> <span class="text-danger font-semibold">RX</span> <span> 6.7 </span>/s <span> 6.7 </span>KB/s</div>
      <div class="flex gap-2 long-bar"> <span class="text-muted">Audio</span> <span class="text-danger font-semibold">RX</span> <span> 6.7 </span>/s <span> 6.7 </span>KB/s</div>
      <div class="flex gap-2 long-bar"> <span class="text-muted w-12"></span> <span class="text-success font-semibold">TX</span> <span> 6.7 </span>/s <span> 6.7 </span>KB/s</div>
    <!-- </div> -->

    <div class="flex flex-2">
      <img src="/Tabletop_microphone_Pinhead_icon.svg" alt="Microphone" class="astro-status-icon">
      <img src="/Speaker_Icon.svg" alt="Speaker" class="astro-status-icon">
    </div>
  </div>
  
</template>

<script lang="ts" setup>

</script>

<style scoped>

.astro-status-icon {
  max-width: 50%;
  max-height: 50%;
}


.speaker-controls-box {
  width:auto;
}

.long-bar {
  grid-column: 1 / 4; 
  margin: 0px;
}

</style>
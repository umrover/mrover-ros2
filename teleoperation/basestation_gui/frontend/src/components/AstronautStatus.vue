<template>
  <!-- <div class="flex-1 flex"> -->
  <h1>Astronaut Status</h1>
  <!-- </div> -->

  <div class="flex-1 flex">
    <div class="border-2 rounded px-1 ml-1 flex flex-col justify-center">
        <!-- <div class="flex gap-2"><span class="text-success font-semibold">TX</span> <span v-html="formatNumber(txMsgRate, 4, 0)"></span>/s <span v-html="formatNumber(txByteRate / 1024, 4, 2)"></span>KB/s</div>
        <div class="flex gap-2"><span class="text-danger font-semibold">RX</span> <span v-html="formatNumber(rxMsgRate, 4, 0)"></span>/s <span v-html="formatNumber(rxByteRate / 1024, 4, 2)"></span>KB/s</div> -->
        <!-- <div class="flex gap-2"><span class="text-success font-semibold">TX</span> <span>6.7</span>/s <span>6.7</span>KB/s</div> -->

        <div class="flex gap-2"> <span class="text-muted">Video</span> <span class="text-danger font-semibold">RX</span> <span>6.7</span>/s <span>6.7</span>KB/s</div>
        <div class="flex gap-2"> <span class="text-muted">Audio</span> <span class="text-danger font-semibold">RX</span> <span>6.7</span>/s <span>6.7</span>KB/s</div>
      </div>

    <!-- <div class="flex-1 flex"> -->
      <img src="/Tabletop_microphone_Pinhead_icon.svg" alt="Microphone" class="mic-icon">
      <img src="/Speaker_Icon.svg" alt="Speaker" class="speaker-icon">
    <!-- </div> -->
  </div>
  
</template>

<script lang="ts" setup>

</script>

<style scoped>

.speaker-icon {
  width: 10%;
  object-fit: contain;
}

.mic-icon {
  width: 10%;
  object-fit: contain;
}

</style>
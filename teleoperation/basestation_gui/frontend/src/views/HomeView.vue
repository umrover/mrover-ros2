<template>
  <div class="flex justify-center items-center p-6 flex-1 min-h-screen bg-theme-view">
    <div class="w-full bg-theme-card border rounded-sm shadow-lg" style="max-width: 600px; border-color: var(--cmd-panel-border) !important;">
      <div class="p-6" style="font-family: var(--cmd-font-mono);">
        <div class="flex items-center flex-wrap mb-4">
          <span class="text-cmd-success">rover@basestation</span><span>:</span><span class="text-muted">~</span><span class="mr-2">$</span>
          <span>./select_mission</span><span class="font-bold">_</span>
        </div>

        <div class="flex flex-col gap-1 pl-4">
          <MenuButton link="/DMTask" name="Delivery" />
          <MenuButton link="/ESTask" name="Equipment Servicing" />
          <MenuButton link="/ScienceTask" name="Science" />
          <MenuButton link="/AutonTask" name="Autonomy" />
        </div>
      </div>
    </div>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'
import MenuButton from '../components/MenuButton.vue'

export default defineComponent({
  components: {
    MenuButton
  },
  setup() {
    return {}
  }
})
</script>


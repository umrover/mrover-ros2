<template>
  <div class="d-flex justify-content-center align-items-center p-4 flex-fill min-vh-100 bg-theme-view">
    <div class="w-100 bg-theme-card border rounded-sm shadow-lg" style="max-width: 600px; border-color: var(--cmd-panel-border) !important;">
      <div class="p-4" style="font-family: var(--cmd-font-mono);">
        <div class="d-flex align-items-center flex-wrap mb-3">
          <span class="user">rover@basestation</span><span class="separator">:</span><span class="path">~</span><span class="prompt-symbol">$</span>
          <span class="command">./select_mission</span><span class="cursor">_</span>
        </div>

        <div class="d-flex flex-column gap-1 ps-3">
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

<style scoped>
.user {
  color: var(--cmd-status-ok);
}

.separator {
  color: var(--text-primary);
}

.path {
  color: var(--text-muted);
}

.prompt-symbol {
  margin-right: 0.5rem;
  color: var(--text-primary);
}

.command {
  color: var(--text-primary);
}

.cursor {
  font-weight: bold;
  color: var(--text-primary);
}
</style>

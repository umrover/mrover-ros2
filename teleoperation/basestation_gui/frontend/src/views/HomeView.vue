<template>
  <div class="wrapper d-flex justify-content-center align-items-center p-4">
    <div class="i3-window shadow">
      <div class="i3-titlebar d-flex align-items-center px-1 pt-0">
        <span class="i3-title">MRover Basestation</span>
      </div>
      <div class="i3-content p-4 d-flex flex-column justify-content-center">
        <div class="terminal-prompt mb-3">
          <span class="user">rover@basestation</span><span class="separator">:</span><span class="path">~</span><span class="prompt-symbol">$</span> <span class="command">./select_mission</span>
        </div>
        <div class="d-flex flex-column gap-3">
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
import MenuButton from '../components/MenuButton.vue'
export default {
  components: {
    MenuButton
  }
}
</script>

<style scoped>
.wrapper {
  flex: 1;
  position: relative;
  overflow: hidden;
  background: var(--view-bg);
}

.i3-window {
  width: 100%;
  max-width: clamp(500px, 36vw, 800px);
  background: var(--card-bg);
  border: clamp(3px, 0.25vw, 5px) solid #4c7899;
  overflow: hidden;
  animation: fadeIn 0.3s ease-out;
  font-family: var(--cmd-font-mono);
  font-size: 1rem;
}

@keyframes fadeIn {
  from { opacity: 0; }
  to { opacity: 1; }
}

.i3-titlebar {
  background: #4c7899;
  border-bottom: 2px solid #4c7899;
  min-height: clamp(24px, 1.5vw, 36px);
  padding: 0.25rem var(--cmd-padding-sm);
}

.i3-title {
  color: #ffffff;
  font-size: 0.875rem;
  font-weight: normal;
}

.i3-content {
  background: var(--card-bg);
  min-height: clamp(250px, 20vw, 400px);
}

.terminal-prompt {
  font-size: 1rem;
  line-height: 1.5;
}

.user {
  color: #2e7d32;
}

.separator {
  color: var(--text-primary);
}

.path {
  color: #1565c0;
  font-weight: bold;
}

.prompt-symbol {
  color: var(--text-primary);
  margin: 0 var(--cmd-gap-sm) 0 0.25rem;
}

.command {
  color: #6d4c41;
}
</style>

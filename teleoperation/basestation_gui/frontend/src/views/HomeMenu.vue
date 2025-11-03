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
          <MenuButton link="/SPTask" name="Science Payload" />
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
  background: #e8e8e8;
}

.i3-window {
  width: 100%;
  max-width: 700px;
  background: #ffffff;
  border: 4px solid #4c7899;
  overflow: hidden;
  animation: fadeIn 0.3s ease-out;
}

@keyframes fadeIn {
  from {
    opacity: 0;
  }
  to {
    opacity: 1;
  }
}

.i3-titlebar {
  background: linear-gradient(to bottom, #4c7899 0%, #4c7899 100%);
  border-bottom: 1px solid #2f5373;
  min-height: 24px;
  padding-top: 0.25rem;
  padding-bottom: 0.25rem;
}

.i3-title {
  color: #ffffff;
  font-family: monospace;
  font-size: 0.875rem;
  font-weight: normal;
  letter-spacing: 0.5px;
}

.i3-content {
  background: #ffffff;
  min-height: 300px;
  font-family: 'Courier New', monospace;
}

.terminal-prompt {
  color: #0e7490;
  font-size: 1rem;
  line-height: 1.5;
}

.user {
  color: #0e7490;
}

.separator {
  color: #1e293b;
  margin: 0 4px;
}

.path {
  color: #1e40af;
  font-weight: bold;
}

.prompt-symbol {
  color: #1e293b;
  margin: 0 8px 0 4px;
}

.command {
  color: #92400e;
}
</style>

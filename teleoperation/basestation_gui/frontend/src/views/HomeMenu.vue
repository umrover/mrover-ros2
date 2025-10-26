<template>
  <div class="wrapper">
    <div class="terminal-container">
      <div class="terminal-header">
        <span class="terminal-title">MRover Basestation</span>
        <div class="terminal-controls">
          <span class="control red"></span>
          <span class="control yellow"></span>
          <span class="control green"></span>
        </div>
      </div>
      <div class="terminal-body">
        <div class="terminal-prompt mb-3">
          <span class="user">rover@basestation</span><span class="separator">:</span><span class="path">~</span><span class="prompt-symbol">$</span> <span class="command">./select_mission</span>
        </div>
        <div class="menu-buttons">
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
  display: flex;
  justify-content: center;
  align-items: center;
  position: relative;
  overflow: hidden;
  background: linear-gradient(135deg, #f0f4f8 0%, #e2e8f0 100%);
  padding: 2rem;
}

.terminal-container {
  width: 100%;
  max-width: 700px;
  background: #ffffff;
  border-radius: 12px;
  box-shadow: 0 20px 60px rgba(0, 0, 0, 0.15);
  overflow: hidden;
  animation: slideIn 0.5s ease-out;
  border: 1px solid #e2e8f0;
}

@keyframes slideIn {
  from {
    opacity: 0;
    transform: translateY(-20px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

.terminal-header {
  background: #f8fafc;
  padding: 0.75rem 1rem;
  display: flex;
  justify-content: space-between;
  align-items: center;
  border-bottom: 1px solid #e2e8f0;
}

.terminal-title {
  color: #334155;
  font-family: 'Courier New', monospace;
  font-size: 0.875rem;
  font-weight: 600;
}

.terminal-controls {
  display: flex;
  gap: 0.5rem;
}

.control {
  width: 12px;
  height: 12px;
  border-radius: 50%;
}

.control.red {
  background: #ff5f56;
}

.control.yellow {
  background: #ffbd2e;
}

.control.green {
  background: #27c93f;
}

.terminal-body {
  padding: 2rem;
  background: #f1f5f9;
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

@keyframes blink {
  0%, 50%, 100% {
    opacity: 1;
  }
  25%, 75% {
    opacity: 0.5;
  }
}

.menu-buttons {
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
}
</style>

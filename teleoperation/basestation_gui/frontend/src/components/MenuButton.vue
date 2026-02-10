<template>
  <a class="menu-button border border-2 rounded" :href="link">
    <span class="terminal-arrow">&gt;</span>
    <span class="button-text">{{ name }}</span>
    <span class="button-bg"></span>
  </a>
</template>

<script lang="ts">
import { defineComponent } from 'vue'

export default defineComponent({
  name: 'MenuButton',

  props: {
    link: {
      type: String,
      required: true
    },
    name: {
      type: String,
      required: true
    }
  },
})
</script>

<style scoped>
a {
  margin: 5px;
  width: 250px;
  height: 150px;

  display: flex;
  justify-content: center;
  align-items: center;
  padding: 1rem clamp(1rem, 1.2vw, 1.75rem);
  text-decoration: none;
  font-family: var(--cmd-font-mono);
  font-size: 1.1rem;
  font-weight: 500;
  color: #4c7899;
  background: transparent;
  border-color: #94a3b8;
  cursor: pointer;
  overflow: hidden;
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
  min-height: 3.5rem;
}

.terminal-arrow {
  position: absolute;
  left: clamp(-25px, -1.5vw, -35px);
  color: #92400e;
  font-size: 1.25rem;
  font-weight: bold;
  opacity: 0;
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
  z-index: 3;
}

.button-text {
  position: relative;
  z-index: 3;
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
}

.button-bg {
  position: absolute;
  top: 0;
  left: 0;
  width: 0;
  height: 100%;
  background: linear-gradient(90deg, rgba(14, 116, 144, 0.18) 0%, rgba(14, 116, 144, 0.1) 100%);
  transition: width 0.4s cubic-bezier(0.4, 0, 0.2, 1);
  z-index: 1;
}

.menu-button:hover {
  border-color: #0e7490;
  padding-left: clamp(2.5rem, 2.5vw, 3.5rem);
  box-shadow: 0 4px 20px rgba(14, 116, 144, 0.25);
  background: rgba(14, 116, 144, 0.08);
}

.menu-button:hover .terminal-arrow {
  left: 1rem;
  opacity: 1;
}

.menu-button:hover .button-text {
  color: #2f5373;
  font-weight: 700;
}

.menu-button:hover .button-bg {
  width: 100%;
}

.menu-button:active {
  transform: scale(0.98);
  box-shadow: 0 2px 10px rgba(14, 116, 144, 0.3);
}
</style>

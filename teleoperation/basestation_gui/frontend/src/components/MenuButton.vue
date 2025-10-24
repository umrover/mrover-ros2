<template>
  <a class="menu-button" :href="link">
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
.menu-button {
  position: relative;
  display: flex;
  align-items: center;
  padding: 1rem 1.5rem;
  text-decoration: none;
  font-family: 'Courier New', monospace;
  font-size: 1.125rem;
  font-weight: 600;
  color: #0e7490;
  background: transparent;
  border: 2px solid #94a3b8;
  border-radius: 6px;
  cursor: pointer;
  overflow: hidden;
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
}

.terminal-arrow {
  position: absolute;
  left: -30px;
  color: #92400e;
  font-size: 1.5rem;
  font-weight: bold;
  opacity: 0;
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
}

.button-text {
  position: relative;
  z-index: 2;
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
  padding-left: 2.5rem;
  box-shadow: 0 4px 20px rgba(14, 116, 144, 0.25);
  background: rgba(14, 116, 144, 0.08);
}

.menu-button:hover .terminal-arrow {
  left: 1rem;
  opacity: 1;
  animation: pulse 1.5s infinite;
}

.menu-button:hover .button-text {
  color: #164e63;
  font-weight: 700;
}

.menu-button:hover .button-bg {
  width: 100%;
}

.menu-button:active {
  transform: scale(0.98);
  box-shadow: 0 2px 10px rgba(14, 116, 144, 0.3);
}

@keyframes pulse {
  0%, 100% {
    transform: translateX(0);
    opacity: 1;
  }
  50% {
    transform: translateX(3px);
    opacity: 0.8;
  }
}

/* Add scanline effect */
.menu-button::before {
  content: '';
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  background: linear-gradient(
    to bottom,
    transparent 50%,
    rgba(14, 116, 144, 0.05) 50%
  );
  background-size: 100% 4px;
  pointer-events: none;
  opacity: 0;
  transition: opacity 0.3s;
}

.menu-button:hover::before {
  opacity: 1;
}

/* Glitch effect on active */
@keyframes glitch {
  0% {
    transform: translate(0);
  }
  20% {
    transform: translate(-2px, 2px);
  }
  40% {
    transform: translate(-2px, -2px);
  }
  60% {
    transform: translate(2px, 2px);
  }
  80% {
    transform: translate(2px, -2px);
  }
  100% {
    transform: translate(0);
  }
}

.menu-button:active .button-text {
  animation: glitch 0.3s;
}
</style>

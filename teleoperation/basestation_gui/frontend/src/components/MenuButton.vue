<template>
  <a class="d-block text-decoration-none py-1 terminal-link" :href="link">
    <span class="me-2 opacity-50 prefix">./</span>{{ name }}
  </a>
</template>

<script lang="ts">
import { defineComponent } from 'vue'

export default defineComponent({
  name: 'MenuButton',
  props: {
    link: { type: String, required: true },
    name: { type: String, required: true }
  },
})
</script>

<style scoped>
.terminal-link {
  font-family: var(--cmd-font-mono);
  font-size: 1rem;
  color: var(--text-muted);
  transition: color 0.1s ease;
}

.prefix {
  color: var(--text-muted);
}

.terminal-link:hover {
  color: var(--text-primary);
}

.terminal-link:hover .prefix {
  color: var(--text-primary);
  opacity: 1 !important;
}

.terminal-link::before {
  margin-right: 0.5rem;
  color: var(--text-primary);
  content: '>';
  opacity: 0;
}

.terminal-link:hover::before {
  opacity: 1;
}
</style>

<template>
  <Teleport to="body">
    <div v-if="active" class="vim-command-bar">
      <span class="vim-command-prompt">:</span>
      <input
        ref="inputRef"
        class="vim-command-input"
        v-model="text"
        @keydown.enter.prevent.stop="execute"
        @keydown.escape.prevent.stop="close"
      />
    </div>
  </Teleport>
</template>

<script lang="ts" setup>
import { ref, nextTick } from 'vue'

const emit = defineEmits<{
  execute: [command: string]
  close: []
}>()

const active = ref(false)
const text = ref('')
const inputRef = ref<HTMLInputElement | null>(null)

function open() {
  active.value = true
  text.value = ''
  nextTick(() => inputRef.value?.focus())
}

function close() {
  active.value = false
  text.value = ''
  emit('close')
}

function execute() {
  const cmd = text.value.trim()
  close()
  if (cmd) emit('execute', cmd)
}

defineExpose({ open, close, active })
</script>

<style>
.vim-command-bar {
  position: fixed;
  bottom: 0;
  left: 0;
  right: 0;
  display: flex;
  align-items: center;
  padding: 0.5rem 0.75rem;
  background-color: #1e1e2e;
  border-top: 2px solid var(--status-ok);
  z-index: 99999;
}

.vim-command-prompt {
  font-family: monospace;
  font-size: 1rem;
  font-weight: 700;
  color: var(--status-ok);
  margin-right: 0.25rem;
}

.vim-command-input {
  flex: 1;
  font-family: monospace;
  font-size: 1rem;
  color: #fff;
  background: transparent;
  border: none;
  outline: none;
}
</style>

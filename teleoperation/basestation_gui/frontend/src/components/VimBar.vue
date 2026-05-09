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
  padding: 0.25rem 0.75rem;
  font-size: 0.75rem;
  font-weight: 600;
  letter-spacing: 0.05em;
  color: #fff;
  background-color: var(--status-ok);
  border-top: 2px solid var(--status-ok);
  z-index: 99999;
}

.vim-command-prompt {
  font-family: monospace;
  font-size: 0.75rem;
  font-weight: 700;
  color: #fff;
  margin-right: 0.25rem;
}

.vim-command-input {
  flex: 1;
  font-family: monospace;
  font-size: 0.75rem;
  font-weight: 600;
  color: #fff;
  background: transparent;
  border: none;
  outline: none;
}

.vim-command-input::placeholder {
  color: rgba(255, 255, 255, 0.5);
}
</style>

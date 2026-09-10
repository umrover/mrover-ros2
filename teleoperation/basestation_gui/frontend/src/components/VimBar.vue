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
  right: 0;
  bottom: 0;
  left: 0;
  z-index: 99999;
  display: flex;
  align-items: center;
  padding: 0.25rem 0.75rem;
  font-size: 0.75rem;
  font-weight: 600;
  color: #fff;
  letter-spacing: 0.05em;
  background-color: var(--status-ok);
  border-top: 2px solid var(--status-ok);
}

.vim-command-prompt {
  margin-right: 0.25rem;
  font-family: monospace;
  font-size: 0.75rem;
  font-weight: 700;
  color: #fff;
}

.vim-command-input {
  flex: 1;
  font-family: monospace;
  font-size: 0.75rem;
  font-weight: 600;
  color: #fff;
  outline: none;
  background: transparent;
  border: none;
}

.vim-command-input::placeholder {
  color: rgb(255 255 255 / 50%);
}
</style>

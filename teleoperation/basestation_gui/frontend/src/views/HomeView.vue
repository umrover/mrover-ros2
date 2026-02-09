<template>
  <div class="d-flex justify-content-center align-items-center p-4 flex-fill min-vh-100 bg-theme-view">
    <div class="w-100 bg-theme-card border rounded-sm shadow-lg fade-in" style="max-width: 600px; border-color: var(--cmd-panel-border) !important;">
      <div class="p-4" style="font-family: var(--cmd-font-mono);">
        <div class="d-flex align-items-center flex-wrap mb-3">
          <span class="user">rover@basestation</span><span class="separator">:</span><span class="path">~</span><span class="prompt-symbol">$</span>
          <span class="command">{{ typedCommand }}</span><span class="cursor">_</span>
        </div>

        <div class="d-flex flex-column gap-1 ps-3 menu-list" :class="{ 'visible': showMenu }">
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
import { defineComponent, ref, onMounted } from 'vue'
import MenuButton from '../components/MenuButton.vue'

export default defineComponent({
  components: {
    MenuButton
  },
  setup() {
    const fullCommand = './select_mission'
    const typedCommand = ref('')
    const showMenu = ref(false)

    onMounted(() => {
      let i = 0
      const typeInterval = setInterval(() => {
        if (i < fullCommand.length) {
          typedCommand.value += fullCommand.charAt(i)
          i++
        } else {
          clearInterval(typeInterval)
          showMenu.value = true
        }
      }, 20)
    })

    return {
      typedCommand,
      showMenu
    }
  }
})
</script>

<style scoped>
.fade-in {
  animation: fadeIn 0.3s ease-out;
}

@keyframes fadeIn {
  from { opacity: 0; transform: translateY(10px); }
  to { opacity: 1; transform: translateY(0); }
}

.user { color: var(--cmd-status-ok); }
.separator { color: var(--text-primary); }
.path { color: var(--text-muted); }
.prompt-symbol { color: var(--text-primary); margin-right: 0.5rem; }
.command { color: var(--text-primary); }

.cursor {
  animation: blink 1s step-end infinite;
  color: var(--text-primary);
  font-weight: bold;
}

@keyframes blink {
  0%, 100% { opacity: 1; }
  50% { opacity: 0; }
}

.menu-list {
  opacity: 0;
  transition: opacity 0.1s ease-out;
}

.menu-list.visible {
  opacity: 1;
}
</style>

<template>
  <button :class="['cmd-btn', currentColor]" :style="customStyles" @click="handleClick">
    <span class="flex items-center w-full">
      <span>{{ displayName }}</span>
      <i v-if="mode === 'toggle'" class="ml-auto" :class="checked ? 'bi bi-check-square-fill' : 'bi bi-square'"></i>
      <i v-else-if="isWaiting" class="bi bi-arrow-repeat animate-spin ml-auto"></i>
    </span>
  </button>
</template>

<script lang="ts">
import { defineComponent, type PropType } from 'vue'
import { useNotificationsStore } from '@/stores/notifications'

export default defineComponent({
  setup() {
    const notificationsStore = useNotificationsStore()
    return { notificationsStore }
  },
  props: {
    name: {
      type: String,
      required: true,
    },
    checked: {
      type: Boolean,
      default: false,
    },
    action: {
      type: Function as PropType<(newState: boolean) => Promise<{ status: string; message?: string }>>,
      required: false,
      default: null,
    },
    width: {
      type: [String, Number],
      default: '',
    },
    height: {
      type: [String, Number],
      default: '',
    },
    mode: {
      type: String as PropType<'toggle' | 'action'>,
      default: 'toggle',
    },
    activeText: {
      type: String,
      default: '',
    },
    completeText: {
      type: String,
      default: '',
    },
  },

  data() {
    return {
      isWaiting: false,
      isComplete: false,
    }
  },

  computed: {
    currentColor(): string {
      if (this.mode === 'action') {
        if (this.isComplete) return 'cmd-btn-success'
        if (this.isWaiting) return 'cmd-btn-warning'
        return 'cmd-btn-success'
      }
      if (this.isWaiting) return 'cmd-btn-warning'
      return this.checked ? 'cmd-btn-success' : 'cmd-btn-danger'
    },

    displayName(): string {
      if (this.mode === 'action') {
        if (this.isComplete && this.completeText) return this.completeText
        if (this.isWaiting) return this.activeText || this.name
        return this.name
      }
      if (this.isWaiting) {
        return `Setting to ${this.checked ? 'ON' : 'OFF'}`
      }
      return this.name
    },

    customStyles() {
      const styles: { [key: string]: string } = {}
      if (this.width) styles.width = typeof this.width === 'number' ? `${this.width}px` : this.width
      if (this.height) styles.height = typeof this.height === 'number' ? `${this.height}px` : this.height
      return styles
    },
  },

  methods: {
    async handleClick() {
      if (this.isWaiting) return

      if (this.mode === 'action') {
        await this.handleActionMode()
      } else {
        await this.handleToggleMode()
      }
    },

    async handleActionMode() {
      if (!this.action) {
        this.$emit('start')
        return
      }

      this.isWaiting = true
      this.$emit('start')

      try {
        const response = await this.action(true)

        if (response.status === 'error') {
          console.error(`${this.name} action failed:`, response.message)
          this.notificationsStore.addNotification({
            component: this.name,
            message: response.message || 'Action failed',
            fullData: response
          })
        } else if (this.completeText) {
          this.isComplete = true
          setTimeout(() => { this.isComplete = false }, 2000)
        }
        this.$emit('complete', response)
      } catch {
        this.$emit('error')
      } finally {
        this.isWaiting = false
      }
    },

    async handleToggleMode() {
      const targetState = !this.checked

      if (!this.action) {
        this.$emit('toggle', targetState)
        return
      }

      this.isWaiting = true
      this.$emit('toggle', targetState)

      try {
        const response = await this.action(targetState)

        if (response.status === 'error') {
          console.error(`${this.name} command failed:`, response.message)
          this.notificationsStore.addNotification({
            component: this.name,
            message: response.message || 'Command failed',
            fullData: response
          })
          this.$emit('toggle', !targetState)
        }
      } catch {
        this.$emit('toggle', !targetState)
      } finally {
        this.isWaiting = false
      }
    },
  },
})
</script>


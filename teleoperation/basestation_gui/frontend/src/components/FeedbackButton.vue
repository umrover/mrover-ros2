<template>
  <button :class="['btn', currentColor]" :style="customStyles" @click="handleClick">
    <span class="d-inline-flex align-items-center gap-2">
      <span>{{ displayName }}</span>
      <i :class="checked ? 'bi bi-check-square-fill' : 'bi bi-square'"></i>
    </span>
  </button>
</template>

<script lang="ts">
import { defineComponent, type PropType } from 'vue'

export default defineComponent({
  props: {
    name: {
      type: String,
      required: true,
    },
    checked: {
      type: Boolean,
      required: true,
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
  },

  data() {
    return {
      isWaiting: false,
    }
  },

  computed: {
    currentColor(): string {
      if (this.isWaiting) {
        return 'btn-warning'
      }
      return this.checked ? 'btn-success' : 'btn-danger'
    },

    displayName(): string {
      if (this.isWaiting) {
        return `Setting to ${!this.checked ? 'ON' : 'OFF'}`
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
      if (this.isWaiting) return // Prevent double-clicks

      const targetState = !this.checked

      // If no action (simple toggle), just emit
      if (!this.action) {
        this.$emit('toggle', targetState)
        return
      }

      // With action, show waiting state
      this.isWaiting = true

      // Emit toggle immediately so parent updates state
      this.$emit('toggle', targetState)

      try {
        const response = await this.action(targetState)

        if (response.status === 'error') {
          console.error(`${this.name} command failed:`, response.message)

          // Log to notification store
          this.$store.commit('notifications/addNotification', {
            component: this.name,
            errorType: 'API Error',
            message: response.message || 'Command failed',
            fullData: response
          })

          // Revert state on error
          this.$emit('toggle', !targetState)
        }
        // On success, state stays as is
      } catch (error) {
        console.error(`${this.name} command failed:`, error)

        // Log to notification store
        this.$store.commit('notifications/addNotification', {
          component: this.name,
          errorType: 'Exception',
          message: error instanceof Error ? error.message : String(error),
          fullData: error instanceof Error ? { message: error.message, stack: error.stack } : { error: String(error) }
        })

        // Revert state on exception
        this.$emit('toggle', !targetState)
      } finally {
        this.isWaiting = false
      }
    },
  },
})
</script>

<style scoped>
</style>

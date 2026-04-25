<template>
<<<<<<< HEAD
  <button :class="['cmd-btn', currentColor]" :style="customStyles" :disabled="disabled" @click="handleClick">
=======
  <button :class="['btn', currentColor]" :style="customStyles" :disabled="disabled" @click="handleClick">
>>>>>>> origin/main
    <span class="flex items-center w-full">
      <span>{{ displayName }}</span>
      <i v-if="mode === 'toggle'" class="ml-auto" :class="checked ? 'bi bi-check-square-fill' : 'bi bi-square'"></i>
      <i v-else-if="isWaiting" class="bi bi-arrow-repeat animate-spin ml-auto"></i>
    </span>
  </button>
</template>

<script lang="ts" setup>
import { ref, computed } from 'vue'
import { useNotificationsStore } from '@/stores/notifications'

<<<<<<< HEAD
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
    disabled: {
      type: Boolean,
      default: false,
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
=======
interface ActionResponse {
  status: string
  message?: string
}

const props = withDefaults(defineProps<{
  name: string
  checked?: boolean
  action?: ((newState: boolean) => Promise<ActionResponse>) | null
  width?: string | number
  height?: string | number
  disabled?: boolean
  mode?: 'toggle' | 'action'
  activeText?: string
  completeText?: string
}>(), {
  checked: false,
  action: null,
  width: '',
  height: '',
  disabled: false,
  mode: 'toggle',
  activeText: '',
  completeText: '',
>>>>>>> origin/main
})

const emit = defineEmits<{
  start: []
  complete: [response: ActionResponse]
  error: []
  toggle: [state: boolean]
}>()

const notificationsStore = useNotificationsStore()

const isWaiting = ref(false)
const isComplete = ref(false)

const currentColor = computed(() => {
  if (props.mode === 'action') {
    if (isComplete.value) return 'btn-success'
    if (isWaiting.value) return 'btn-warning'
    return 'btn-success'
  }
  if (isWaiting.value) return 'btn-warning'
  return props.checked ? 'btn-success' : 'btn-danger'
})

const displayName = computed(() => {
  if (props.mode === 'action') {
    if (isComplete.value && props.completeText) return props.completeText
    if (isWaiting.value) return props.activeText || props.name
    return props.name
  }
  if (isWaiting.value) {
    return `Setting to ${props.checked ? 'ON' : 'OFF'}`
  }
  return props.name
})

const customStyles = computed(() => {
  const styles: Record<string, string> = {}
  if (props.width) styles.width = typeof props.width === 'number' ? `${props.width}px` : props.width
  if (props.height) styles.height = typeof props.height === 'number' ? `${props.height}px` : props.height
  return styles
})

async function handleActionMode() {
  if (!props.action) {
    emit('start')
    return
  }

  isWaiting.value = true
  emit('start')

  try {
    const response = await props.action(true)

    if (response.status === 'error') {
      console.error(`${props.name} action failed:`, response.message)
      notificationsStore.addNotification({
        component: props.name,
        message: response.message || 'Action failed',
        fullData: response
      })
    } else if (props.completeText) {
      isComplete.value = true
      setTimeout(() => { isComplete.value = false }, 2000)
    }
    emit('complete', response)
  } catch {
    emit('error')
  } finally {
    isWaiting.value = false
  }
}

async function handleToggleMode() {
  const targetState = !props.checked

  if (!props.action) {
    emit('toggle', targetState)
    return
  }

  isWaiting.value = true
  emit('toggle', targetState)

  try {
    const response = await props.action(targetState)

    if (response.status === 'error') {
      console.error(`${props.name} command failed:`, response.message)
      notificationsStore.addNotification({
        component: props.name,
        message: response.message || 'Command failed',
        fullData: response
      })
      emit('toggle', !targetState)
    }
  } catch {
    emit('toggle', !targetState)
  } finally {
    isWaiting.value = false
  }
}

async function handleClick() {
  if (isWaiting.value) return

  if (props.mode === 'action') {
    await handleActionMode()
  } else {
    await handleToggleMode()
  }
}
</script>


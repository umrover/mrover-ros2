<template>
  <div v-if="!disabled" class="wrap-button">
    <button
      :class="['btn', color]"
      :style="customStyles"
      @click="toggleAndEmit()"
    >
      <span>{{ name }}: {{ active ? '\u2611' : '\u2610' }}</span>
    </button>
  </div>
  <div v-else class="wrap-button button-disabled">
    <button
      :class="['btn', color, 'disabled']"
      :style="customStyles"
    >
      <span>{{ name }}: {{ active ? '\u2611' : '\u2610' }}</span>
    </button>
  </div>
</template>

<script lang="ts">
import { defineComponent } from 'vue'

export default defineComponent({
  props: {
    name: {
      type: String,
      required: true,
    },
    disabled: {
      type: Boolean,
      default: false,
      required: false,
    },
    height: {
      type: [String, Number],
      default: '',
    },
    width: {
      type: [String, Number],
      default: '',
    },
  },
  data() {
    return {
      active: false,
    }
  },

  computed: {
    color() {
      return this.active ? 'btn-success' : 'btn-danger'
    },

    customStyles() {
      const styles: { [key: string]: string | number } = {}
      
      if (this.width) styles.width = `${this.width}px`
      if (this.height) styles.height = `${this.height}px`
      
      return styles
    },
  },

  methods: {
    toggle() {
      this.active = !this.active
    },

    toggleAndEmit() {
      this.toggle()
      this.$emit('toggle', this.active)
    },
  },
})
</script>

<style scoped>
.wrap-button {
  display: flex;
  align-items: center;
  justify-content: center;
  padding: 1px;
}

.button-disabled {
  opacity: 0.5;
}
</style>

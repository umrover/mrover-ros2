<template>
  <div v-if="!disabled" class="d-flex align-items-center justify-content-center p-1">
    <button
      :class="['btn checkbox-btn border-2', color]"
      :style="customStyles"
      @click="toggleAndEmit()"
    >
      <span>{{ name }}</span>
      <i :class="active ? 'bi bi-check-square-fill' : 'bi bi-square'"></i>
    </button>
  </div>
  <div v-else class="d-flex align-items-center justify-content-center p-1 opacity-50">
    <button
      :class="['btn checkbox-btn border-2', color, 'disabled']"
      :style="customStyles"
    >
      <span>{{ name }}</span>
      <i :class="active ? 'bi bi-check-square-fill' : 'bi bi-square'"></i>
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
.checkbox-btn {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  font-size: 0.8125rem;
  font-weight: 500;
  text-transform: uppercase;
  letter-spacing: 0.03em;
  transition: all var(--cmd-transition);
}
</style>

<template>
  <button :class="['btn', color]" @click="handleClick()">
    <span class="white-text d-inline-flex align-items-center">
      <span>{{ name }}: </span>
      <span class="checkbox">{{ checked ? '\u2611' : '\u2610' }}</span>
    </span>
  </button>
</template>

<script lang="ts">
export default {
  props: {
    name: {
      type: String,
      required: true,
    },
    color: {
      type: String,
      required: true,
    },
    checked: {
      type: Boolean,
      required: true,
    },
  },

  methods: {
    handleClick() {
      this.$emit('toggle', !this.checked)
    },
  },
}
</script>

<style scoped>
.wrap-button {
  display: block;
  align-items: center;
  justify-content: center;
  padding: 1px;
}
</style>

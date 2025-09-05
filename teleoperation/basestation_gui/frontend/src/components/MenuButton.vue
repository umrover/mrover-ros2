<template>
  <a class="btn btn-primary" :href="link">{{ name }}</a>
</template>
<script lang="ts">
import { defineComponent } from 'vue'

<script lang="ts">
import { defineComponent } from 'vue'

export default defineComponent({
  name: 'MenuButton',

  props: {
    link: {
      type: String,
      required: true
    },
    name: {
      type: String,
      required: true
    }
  },
})
</script>

<style scoped>
a {
  margin: 5px;
  width: 250px;
  height: 150px;

  display: flex;
  justify-content: center;
  align-items: center;

  text-decoration: none;
  color: #FFCB05;
  font-size: 2rem;
  font-weight: 900;

  background-color: #00274C;
  border-radius: 10px;
  box-shadow: 0 4px 10px rgba(0, 0, 0, 0.1);
  transition: transform 0.2s, box-shadow 0.2s;
}

a:hover {
  transform: translateY(-4px);
  box-shadow: 0 6px 10px rgba(0, 0, 0, 0.2);
}

a:active {
  transform: translateY(0);
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
}

</style>

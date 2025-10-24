<template>
  <div class="wrapper">
    <fieldset class="row">
      <MenuButton link="/DMTask" name="DM Mission" />
      <MenuButton link="/ESTask" name="ES Mission" />
      <MenuButton link="/AutonTask" name="Autonomy Mission" />
      <MenuButton link="/SPTask" name="SP GUI" />
    </fieldset>
  </div>
</template>

<script lang="ts">
import MenuButton from '../components/MenuButton.vue'
export default {
  components: {
    MenuButton
  }
}
</script>

<style scoped>
.wrapper {
  flex: 1;
  display: flex;
  justify-content: center;
  align-items: center;
  position: relative;
  overflow: hidden;
}

.wrapper {
  background-color: #dddddd;
}

.row {
  display: flex;
  flex-wrap: wrap;
  justify-content: center;
  text-align: center;
  margin: 0 0 10rem 0;
}
</style>

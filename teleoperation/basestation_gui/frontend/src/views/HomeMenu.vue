<template>
  <div class="wrapper">
    <fieldset class="row">
      <MenuButton link="/DMTask" name="DM Mission" />
      <MenuButton link="/ESTask" name="ES Mission" />
      <MenuButton link="/ISHTask" name="ISH GUI" />
      <MenuButton link="/SATask" name="SA GUI" />
      <MenuButton link="/AutonTask" name="Autonomy Mission" />
      <MenuButton link="/Cameras" name="Cameras" />
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
  background-color: rgba(230, 120, 60, 0.3);
}

.wrapper::before {
  content: '';
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background-image: url('/menu_bg.jpg');
  background-size: cover;
  background-position: center;
  opacity: 0.2;
  z-index: 0;
}

.wrapper > * {
  position: relative;
  z-index: 1;
}

.row {
  display: flex;
  flex-wrap: wrap;
  justify-content: center;
  text-align: center;
  margin: 0 0 10rem 0;
}
</style>

import { createRouter, createWebHistory } from 'vue-router'
import Menu from '../views/HomeMenu.vue'
import DMESTask from '../views/DMESTask.vue'
import AutonTask from '../views/AutonTask.vue'
import ISHTask from '../views/ISHTask.vue'
import SATask from '../views/SATask.vue'
import CameraView from '../views/CameraView.vue'
import Rover3D from '../components/Rover3D.vue'

const routes = [
  {
    path: '/',
    name: 'Menu',
    component: Menu
  },
  {
    path: '/DMTask',
    name: 'DMTask',
    component: DMESTask,
    props: {
      type: 'DM'
    }
  },
  {
    path: '/ESTask',
    name: 'ESTask',
    component: DMESTask,
    props: {
      type: 'ES'
    }
  },
  {
    path: '/AutonTask',
    name: 'AutonTask',
    component: AutonTask
  },
  {
    path: '/SATask',
    name: 'SATask',
    component: SATask
  },
  {
    path: '/ISHTask',
    name: 'ISHTask',
    component: ISHTask
  },
  {
    path: '/Cameras',
    name: 'Cameras',
    component: CameraView
  },
  {
    path: '/Control',
    name: 'Control',
    component: Rover3D
  }
]
const router = createRouter({
  history: createWebHistory(),
  routes
})

export default router

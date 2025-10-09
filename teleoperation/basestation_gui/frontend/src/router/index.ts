import { createRouter, createWebHistory } from 'vue-router'
import Menu from '../views/HomeMenu.vue'
import DMTask from '../views/DMTask.vue'
import ESTask from '../views/ESTask.vue'
import AutonTask from '../views/AutonTask.vue'
import ISHTask from '../views/ISHTask.vue'
import SATask from '../views/SATask.vue'
import CameraView from '../views/CameraView.vue'
import Rover3D from '../components/Rover3D.vue'
import DevView from '../views/DevView.vue'

const routes = [
  {
    path: '/',
    name: 'Menu',
    component: Menu
  },
  {
    path: '/DMTask',
    name: 'DMTask',
    component: DMTask
  },
  {
    path: '/ESTask',
    name: 'ESTask',
    component: ESTask
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
  },
  {
    path: '/dev',
    name: 'DevView',
    component: DevView
  }
]
const router = createRouter({
  history: createWebHistory(),
  routes
})

export default router

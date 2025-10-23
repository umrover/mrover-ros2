import { createRouter, createWebHistory } from 'vue-router'
import Menu from '../views/HomeMenu.vue'
import DMTask from '../views/DMTask.vue'
import ESTask from '../views/ESTask.vue'
import AutonTask from '../views/AutonTask.vue'
import CameraView from '../views/CameraView.vue'
import Rover3D from '../components/Rover3D.vue'
import DevView from '../views/DevView.vue'
import SPTask from '../views/SPTask.vue'


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
  },
  {
    path: '/SPTask',
    name: 'SPTask',
    component: SPTask
  }
]
const router = createRouter({
  history: createWebHistory(),
  routes
})

export default router

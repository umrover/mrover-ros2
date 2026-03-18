import { createRouter, createWebHistory } from 'vue-router'
import Menu from '@/views/HomeView.vue'
import DMTask from '@/views/DMView.vue'
import ESTask from '@/views/ESView.vue'
import AutonTask from '@/views/AutonView.vue'
import Rover3D from '@/components/Rover3D.vue'
import ScienceTask from '@/views/ScienceView.vue'
import DevView from '@/views/DevView.vue'


const routes = [
  {
    path: '/',
    name: 'Menu',
    component: Menu
  },
  {
    path: '/DMTask',
    name: 'DMTask',
    component: DMTask,
    meta: { hasGridLayout: true }
  },
  {
    path: '/ESTask',
    name: 'ESTask',
    component: ESTask,
    meta: { hasGridLayout: true }
  },
  {
    path: '/AutonTask',
    name: 'AutonTask',
    component: AutonTask,
    meta: { hasGridLayout: true }
  },
  {
    path: '/Control',
    name: 'Control',
    component: Rover3D
  },
  {
    path: '/ScienceTask',
    name: 'ScienceTask',
    component: ScienceTask,
    meta: { hasGridLayout: true }
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

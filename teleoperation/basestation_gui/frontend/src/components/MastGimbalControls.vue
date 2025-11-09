<template>
  <div class='d-flex flex-column gap-2 p-2 border border-2 rounded'>
    <h5 class="m-0">Gimbal Controls</h5>

    <div class="d-flex flex-column gap-1">
      <div class="fw-bold small">Pitch:</div>
      <div class="btn-group btn-group-sm">
        <button class="btn btn-outline-primary" @click="adjustGimbal('pitch', -10)">-10°</button>
        <button class="btn btn-outline-primary" @click="adjustGimbal('pitch', -5)">-5°</button>
        <button class="btn btn-outline-primary" @click="adjustGimbal('pitch', 5)">+5°</button>
        <button class="btn btn-outline-primary" @click="adjustGimbal('pitch', 10)">+10°</button>
      </div>
    </div>

    <div class="d-flex flex-column gap-1">
      <div class="fw-bold small">Yaw:</div>
      <div class="btn-group btn-group-sm">
        <button class="btn btn-outline-primary" @click="adjustGimbal('yaw', -10)">-10°</button>
        <button class="btn btn-outline-primary" @click="adjustGimbal('yaw', -5)">-5°</button>
        <button class="btn btn-outline-primary" @click="adjustGimbal('yaw', 5)">+5°</button>
        <button class="btn btn-outline-primary" @click="adjustGimbal('yaw', 10)">+10°</button>
      </div>
    </div>

    <div class="d-flex flex-column gap-1">
      <div class="fw-bold small">Pitch Presets:</div>
      <div class="btn-group btn-group-sm">
        <button class="btn btn-outline-success" @click="setAbsolute('pitch', -30)">Up</button>
        <button class="btn btn-outline-success" @click="setAbsolute('pitch', 0)">Forward</button>
        <button class="btn btn-outline-success" @click="setAbsolute('pitch', 45)">Down</button>
      </div>
    </div>
  </div>
</template>

<script lang='ts' setup>
import { mastAPI } from '@/utils/mastAPI'

const adjustGimbal = async (joint: 'pitch' | 'yaw', adjustment: number) => {
  try {
    await mastAPI.adjustGimbal(joint, adjustment, false)
  } catch (error) {
    console.error('Failed to adjust gimbal:', error)
  }
}

const setAbsolute = async (joint: 'pitch' | 'yaw', position: number) => {
  try {
    await mastAPI.adjustGimbal(joint, position, true)
  } catch (error) {
    console.error('Failed to set gimbal position:', error)
  }
}
</script>

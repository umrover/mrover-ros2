import type { Quaternion } from '@/types/coordinates';

function quaternionToYaw(quaternion: Quaternion): number {
  const { x: qx, y: qy, z: qz, w: qw } = quaternion;
  return Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
}

function quaternionToMapAngle(quaternion: Quaternion): number {
  return quaternionToYaw(quaternion) * (180 / Math.PI) + 90
}

const ROTATION_OFFSET = Math.PI

export { quaternionToYaw, quaternionToMapAngle, ROTATION_OFFSET }

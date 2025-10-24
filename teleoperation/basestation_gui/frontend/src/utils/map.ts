import type { Quaternion } from '@/types/coordinates';

const quaternionToMapAngle = function (quaternion: Quaternion): number {
  const { x: qx, y: qy, z: qz, w: qw } = quaternion;
  const yaw = Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
  return yaw * (180 / Math.PI)
}

export {quaternionToMapAngle }

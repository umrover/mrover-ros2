const quaternionToMapAngle = function (quaternion: number[]): number {
  const [qx, qy, qz, qw] = quaternion
  const yaw = Math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
  return yaw * (180 / Math.PI)
}

export {quaternionToMapAngle }

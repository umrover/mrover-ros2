declare module '../rover_three.js' {
  export function fk(
    positions: number[],
    scene: import('three').Scene,
    joints: any[]
  ): void

  export function ik(
    target: { quaternion: number[]; position: number[] },
    targetCube: import('three').Mesh
  ): void

  export default function threeSetup(containerId: string): {
    fk: (positions: number[]) => void
    ik: (target: { quaternion: number[]; position: number[] }) => void
  }
}

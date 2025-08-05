export interface Joint {
  name: string
  file: string
  translation: [number, number, number]
  rotation: [number, number, number]
}

export interface IKTarget {
  quaternion: [number, number, number, number]
  position: [number, number, number]
}

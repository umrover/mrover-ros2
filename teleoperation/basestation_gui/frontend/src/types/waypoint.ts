export interface Waypoint {
  name: string
  id: number
  type: number
  lat: number
  lon: number
  enable_costmap: boolean
  in_route?: boolean
}

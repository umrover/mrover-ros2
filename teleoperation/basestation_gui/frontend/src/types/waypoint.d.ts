export interface Waypoint { // for auton
  name: string
  id: number
  type: number
  lat: number
  lon: number
  enable_costmap: boolean
  in_route?: boolean
}

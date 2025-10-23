/**
 * API Request and Response Types
 */

// Waypoint type for auton waypoints
export interface AutonWaypoint {
  name: string
  id: number
  type: number
  lat: number
  lon: number
  enable_costmap: boolean
  in_route?: boolean
}

// Waypoint type for basic waypoints
export interface BasicWaypoint {
  name: string
  lat: number
  lon: number
  drone?: boolean
}

// API Response types
export interface APIResponse {
  status: 'success' | 'error'
  message?: string
}

export interface WaypointsResponse extends APIResponse {
  waypoints?: AutonWaypoint[]
}

export interface BasicWaypointsResponse extends APIResponse {
  waypoints?: BasicWaypoint[]
}

export interface CurrentCourseResponse extends APIResponse {
  course?: AutonWaypoint[]
}

export interface AutonEnableResponse extends APIResponse {
  enabled?: boolean
  waypoint_count?: number
}

export interface TeleopEnableResponse extends APIResponse {
  enabled?: boolean
}

export interface DeleteWaypointResponse extends APIResponse {
  deleted?: boolean
}

// Auton waypoint for enable request (subset of AutonWaypoint)
export interface AutonEnableWaypoint {
  latitude_degrees: number
  longitude_degrees: number
  tag_id: number
  type: number
  enable_costmap: boolean
}

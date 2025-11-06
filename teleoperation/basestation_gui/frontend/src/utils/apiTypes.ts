/**
 * API Request and Response Types
 */

import type { AutonWaypoint, APIBasicWaypoint } from '@/types/waypoints'

// Re-export waypoint types for API usage
export type { AutonWaypoint }
export type BasicWaypoint = APIBasicWaypoint

// API Response types
export interface APIResponse {
  status: 'success' | 'error'
  message?: string
  image_path?: string
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

export interface RAModeResponse extends APIResponse {
  mode: string
}
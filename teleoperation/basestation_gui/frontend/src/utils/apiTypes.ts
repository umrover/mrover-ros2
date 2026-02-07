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
  coverage_radius: number
}

export interface RecordedWaypoint {
  id: number
  lat: number
  lon: number
  timestamp: string
  sequence: number
}

export interface Recording {
  id: number
  name: string
  is_drone: boolean
  created_at: string
  waypoint_count: number
}

export interface RecordingCreateResponse extends APIResponse {
  recording_id?: number
}

export interface RecordingsListResponse extends APIResponse {
  recordings?: Recording[]
}

export interface RecordingWaypointsResponse extends APIResponse {
  waypoints?: RecordedWaypoint[]
}

export interface RAModeResponse extends APIResponse {
  mode?: string
}

// TODO(stow): Add StowResponse type for POST /api/arm/stow/ endpoint.

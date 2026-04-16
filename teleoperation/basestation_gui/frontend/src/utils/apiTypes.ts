import type { AutonWaypoint, BasicWaypointRecord } from '@/types/waypoints'

export type { AutonWaypoint, BasicWaypointRecord }

export interface APIResponse {
  status: 'success' | 'error'
  message?: string
  image_path?: string
}

export interface WaypointsResponse extends APIResponse {
  waypoints?: AutonWaypoint[]
}

export interface BasicWaypointsResponse extends APIResponse {
  waypoints?: BasicWaypointRecord[]
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

export interface CreateWaypointResponse extends APIResponse {
  waypoint?: AutonWaypoint
}

export interface CreateBasicWaypointResponse extends APIResponse {
  waypoint?: BasicWaypointRecord
}

export interface AutonEnableWaypoint {
  latitude_degrees: number
  longitude_degrees: number
  tag_id: number
  type: number
  enable_costmap: boolean
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

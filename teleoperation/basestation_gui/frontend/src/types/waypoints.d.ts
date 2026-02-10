import type { LatLng } from 'leaflet'

/**
 * Waypoint for autonomous navigation missions
 * Includes AprilTag ID, type, and costmap settings
 */
export interface AutonWaypoint {
  name: string
  tag_id: number
  type: number
  lat: number
  lon: number
  enable_costmap: boolean
  coverage_radius: number
  in_route?: boolean
  deletable?: boolean
  db_id?: number
}

/**
 * Basic waypoint for simple map display
 * Used in ERD missions and basic navigation
 */
export interface BasicWaypoint {
  name: string
  latLng: LatLng
  drone?: boolean
}

/**
 * Waypoint representation for internal store state
 * Used by Pinia stores for map waypoint lists
 */
export interface StoreWaypoint {
  name: string
  latLng: LatLng
  drone?: boolean
}

/**
 * Extended waypoint for autonomous navigation routes
 * Includes AprilTag ID, type, and costmap settings alongside LatLng
 */
export interface AutonRouteWaypoint extends StoreWaypoint {
  id: number
  type: number
  enable_costmap: boolean
}

/**
 * API request/response types for waypoints
 */

// Basic waypoint from API (lat/lon instead of LatLng)
export interface APIBasicWaypoint {
  name: string
  lat: number
  lon: number
  drone?: boolean
}

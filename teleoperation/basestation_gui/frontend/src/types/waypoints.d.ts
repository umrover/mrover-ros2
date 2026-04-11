import type { LatLng } from 'leaflet'

export interface AutonWaypoint {
  name: string
  tag_id: number | null
  type: number
  lat: number
  lon: number
  deletable?: boolean
  db_id?: number
}

export interface BasicWaypointRecord {
  db_id?: number
  name: string
  lat: number
  lon: number
  drone?: boolean
}

export interface MapWaypoint {
  latLng: LatLng
  name: string
  drone?: boolean
}

export interface MapRouteWaypoint extends MapWaypoint {
  tag_id: number | null
  type: number
}

import type { LatLng } from 'leaflet';

export interface Waypoint { // for basic
  name: string
  latLng: LatLng
  drone?: boolean
}

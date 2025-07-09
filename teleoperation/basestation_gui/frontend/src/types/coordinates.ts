export interface DMS {
  d: number; // Degrees
  m: number; // Minutes
  s: number; // Seconds
}

export interface FormattedOdom {
  lat: DMS;
  lon: DMS;
}

export interface Odom {
  latitude_deg: number;
  longitude_deg: number;
  bearing_deg: number;
}

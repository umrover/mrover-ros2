export interface SensorData {
  sp_oxygen: number
  sp_uv: number
  sp_humidity: number
  sp_temp: number
  sp_ozone: number
  sp_co2: number
  sp_pressure: number
  sensor_states: SensorStates
}

export interface SensorStates {
  uv_state: boolean
  thp_state: boolean
  oxygen_state: boolean
  ozone_state: boolean
  co2_state: boolean
}
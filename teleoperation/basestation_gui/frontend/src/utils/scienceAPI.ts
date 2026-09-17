import type { ServoResponse } from './apiTypes'
import { apiFetch } from './apiFetch'

export const scienceAPI = {
  setFunnelPosition(position: number, isCounterclockwise: boolean): Promise<ServoResponse> {
    return apiFetch('/science/funnel-servo/position/', {
      method: 'POST',
      body: JSON.stringify({ position, is_counterclockwise: isCounterclockwise })
    })
  }
}

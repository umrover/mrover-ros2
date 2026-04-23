import type { APIResponse } from './apiTypes'
import { apiFetch } from './apiFetch'

export const scienceAPI = {
  setGearDiffPosition(position: number, isCounterclockwise: boolean): Promise<APIResponse> {
    return apiFetch('/science/gear-diff/position/', {
      method: 'POST',
      body: JSON.stringify({ position, is_counterclockwise: isCounterclockwise })
    })
  }
}

import { apiFetch } from './apiFetch'

export const ledAPI = {
  setMission(mission: 'dm' | 'es' | 'science' | 'auton' | 'home') {
    return apiFetch('/led/', {
      method: 'POST',
      body: JSON.stringify({ mission })
    })
  }
}

import { useNotificationsStore } from '@/stores/notifications'

const API_BASE = '/api'

export async function apiFetch<T>(
  endpoint: string,
  options?: RequestInit
): Promise<T> {
  const url = `${API_BASE}${endpoint}`
  const response = await fetch(url, {
    headers: { 'Content-Type': 'application/json', ...options?.headers },
    ...options
  })

  if (!response.ok) {
    const error = await response.json().catch(() => ({ detail: response.statusText }))
    console.log("error.detail")
    console.log(error.detail)
    console.log("response.statusText")
    console.log(response.statusText)
    const message = response.statusText || error.detail
    useNotificationsStore().addAPIError(endpoint, message, response.status)
    throw new Error(message)
  }

  return response.json()
}

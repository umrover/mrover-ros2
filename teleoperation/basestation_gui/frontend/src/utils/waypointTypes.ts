export const WAYPOINT_TYPE = {
  NO_SEARCH: 0,
  POST: 1,
  MALLET: 2,
  WATER_BOTTLE: 3,
  ROCK_PICK: 4,
} as const

export type WaypointTypeValue = (typeof WAYPOINT_TYPE)[keyof typeof WAYPOINT_TYPE]

const TYPE_NAMES: Record<number, string> = {
  [WAYPOINT_TYPE.NO_SEARCH]: 'No Search',
  [WAYPOINT_TYPE.POST]: 'Post',
  [WAYPOINT_TYPE.MALLET]: 'Mallet',
  [WAYPOINT_TYPE.WATER_BOTTLE]: 'Water Bottle',
  [WAYPOINT_TYPE.ROCK_PICK]: 'Rock Pick',
}

export function waypointTypeName(type: number): string {
  return TYPE_NAMES[type] ?? 'Unknown'
}

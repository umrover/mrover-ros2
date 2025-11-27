from pydantic import BaseModel
from typing import List, Optional

class BasicWaypoint(BaseModel):
    name: str
    lat: float
    lon: float
    drone: bool = False

class BasicWaypointList(BaseModel):
    waypoints: List[BasicWaypoint]

class AutonWaypoint(BaseModel):
    name: str
    id: int = -1
    type: int = 0
    lat: float = 0.0
    lon: float = 0.0
    enable_costmap: bool = True
    deletable: bool = True

class AutonWaypointList(BaseModel):
    waypoints: List[AutonWaypoint]

class AutonEnableRequest(BaseModel):
    enabled: bool
    waypoints: List[AutonWaypoint] = []

class TeleopEnableRequest(BaseModel):
    enabled: bool

class GimbalAdjustRequest(BaseModel):
    joint: str
    adjustment: float
    absolute: bool = False

class GearDiffRequest(BaseModel):
    position: float
    is_counterclockwise: bool = False

class RecordingCreateRequest(BaseModel):
    name: str
    is_drone: bool = False

class RecordingWaypointRequest(BaseModel):
    lat: float
    lon: float
    sequence: int

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
    tag_id: int = -1
    type: int = 0
    lat: float = 0.0
    lon: float = 0.0
    enable_costmap: bool = True
    deletable: bool = True
    db_id: int | None = None

class AutonWaypointList(BaseModel):
    waypoints: List[AutonWaypoint]

class AutonEnableWaypoint(BaseModel):
    latitude_degrees: float
    longitude_degrees: float
    tag_id: int = -1
    type: int = 0
    enable_costmap: bool = True

class AutonEnableRequest(BaseModel):
    enabled: bool
    waypoints: List[AutonEnableWaypoint] = []

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

class RAModeRequest(BaseModel):
    mode: str

class ServoPositionRequest(BaseModel):
    names: List[str]
    positions: List[float]

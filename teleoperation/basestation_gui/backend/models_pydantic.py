from pydantic import BaseModel, Field, model_validator
from typing import List, Optional

class BasicWaypoint(BaseModel):
    name: str
    lat: float = Field(ge=-90.0, le=90.0)
    lon: float = Field(ge=-180.0, le=180.0)
    drone: bool = False

class BasicWaypointList(BaseModel):
    waypoints: List[BasicWaypoint]

class AutonWaypoint(BaseModel):
    name: str
    tag_id: int = -1
    type: int = 0
    lat: float = Field(default=0.0, ge=-90.0, le=90.0)
    lon: float = Field(default=0.0, ge=-180.0, le=180.0)
    enable_costmap: bool = True
    deletable: bool = True
    db_id: int | None = None

class AutonWaypointList(BaseModel):
    waypoints: List[AutonWaypoint]

class CreateAutonWaypoint(BaseModel):
    name: str
    tag_id: int = -1
    type: int = 0
    lat: float = Field(default=0.0, ge=-90.0, le=90.0)
    lon: float = Field(default=0.0, ge=-180.0, le=180.0)
    enable_costmap: bool = True

class UpdateAutonWaypoint(BaseModel):
    name: str | None = None
    tag_id: int | None = None
    type: int | None = None
    lat: float | None = Field(default=None, ge=-90.0, le=90.0)
    lon: float | None = Field(default=None, ge=-180.0, le=180.0)
    enable_costmap: bool | None = None

class UpdateBasicWaypoint(BaseModel):
    name: str | None = None
    lat: float | None = Field(default=None, ge=-90.0, le=90.0)
    lon: float | None = Field(default=None, ge=-180.0, le=180.0)
    drone: bool | None = None

class AutonEnableWaypoint(BaseModel):
    latitude_degrees: float = Field(ge=-90.0, le=90.0)
    longitude_degrees: float = Field(ge=-180.0, le=180.0)
    tag_id: int = -1
    type: int = 0
    enable_costmap: bool = True

class AutonEnableRequest(BaseModel):
    enabled: bool
    waypoints: List[AutonEnableWaypoint] = []

class TeleopEnableRequest(BaseModel):
    enabled: bool

class GimbalPositionRequest(BaseModel):
    joint: str
    position: float

class GimbalAdjustRequest(BaseModel):
    joint: str
    adjustment: float = Field(ge=-3.14159, le=3.14159)
    absolute: bool = False

class ServoPositionCommand(BaseModel):
    position: float = Field(ge=0.0, le=6.28319)
    is_counterclockwise: bool = False

class RecordingCreateRequest(BaseModel):
    name: str
    is_drone: bool = False

class RecordingWaypointRequest(BaseModel):
    lat: float = Field(ge=-90.0, le=90.0)
    lon: float = Field(ge=-180.0, le=180.0)
    sequence: int

class RAModeRequest(BaseModel):
    mode: str

class ServoPositionRequest(BaseModel):
    names: List[str]
    positions: List[float]

    @model_validator(mode='after')
    def check_lengths_match(self):
        if len(self.names) != len(self.positions):
            raise ValueError(f'names length ({len(self.names)}) != positions length ({len(self.positions)})')
        return self

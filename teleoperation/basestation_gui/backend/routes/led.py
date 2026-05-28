from fastapi import APIRouter
from pydantic import BaseModel
from backend.managers.led import set_mission

router = APIRouter(prefix="/api", tags=["led"])


class LEDRequest(BaseModel):
    mission: str


@router.post("/led/")
async def set_led(data: LEDRequest):
    set_mission(data.mission)
    return {}

from fastapi import APIRouter, HTTPException
from backend.ra_controls import (
    set_ra_mode as update_ra_mode,
    STOW_POSITION,
    stow_position_dict,
    update_stow_position,
    capture_current_arm_pose,
    load_stow_position,
)
from backend.models_pydantic import RAModeRequest, StowPositionRequest
from backend.database import reset_config_db

router = APIRouter(prefix="/api/arm", tags=["arm"])

VALID_RA_MODES = ["disabled", "throttle", "ik-pos", "ik-vel", "stow"]


@router.post("/ra_mode/")
async def change_ra_mode(data: RAModeRequest):
    mode = data.mode
    if mode not in VALID_RA_MODES:
        raise HTTPException(
            status_code=400, detail=f"Invalid mode '{mode}'. Must be one of: {', '.join(VALID_RA_MODES)}"
        )

    if not await update_ra_mode(mode):
        raise HTTPException(status_code=503, detail="Failed to set RA mode (ROS service might be unavailable)")

    return {"status": "success", "mode": mode}


@router.post("/stow/")
async def change_to_stow():
    mode = "stow"
    if not await update_ra_mode(mode):
        raise HTTPException(status_code=503, detail="Failed to start stow sequence (ROS service might be unavailable)")

    return {
        "status": "success",
        "mode": mode,
        "stow_target": {
            "pos": {
                "x": STOW_POSITION.pos.x,
                "y": STOW_POSITION.pos.y,
                "z": STOW_POSITION.pos.z,
            },
            "pitch": STOW_POSITION.pitch,
            "roll": STOW_POSITION.roll,
        },
    }


@router.get("/stow/config/")
async def get_stow_config():
    return {"status": "success", "stow_position": stow_position_dict()}


@router.post("/stow/capture/")
async def capture_stow_pose():
    pose = capture_current_arm_pose()
    if pose is None:
        raise HTTPException(
            status_code=503,
            detail="TF transform arm_base_link -> arm_fk is not available yet.",
        )
    return {"status": "success", "stow_position": pose}


@router.post("/stow/config/")
async def save_stow_config(data: StowPositionRequest):
    saved = update_stow_position(data.x, data.y, data.z, data.pitch, data.roll)
    return {"status": "success", "stow_position": saved}


@router.post("/stow/config/reset/")
async def reset_stow_config():
    reset_config_db()
    load_stow_position()
    return {"status": "success", "stow_position": stow_position_dict()}

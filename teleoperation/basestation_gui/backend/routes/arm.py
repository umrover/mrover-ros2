from fastapi import APIRouter, HTTPException
from backend.ra_controls import set_ra_mode as update_ra_mode, STOW_POSITION
from backend.models_pydantic import RAModeRequest

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
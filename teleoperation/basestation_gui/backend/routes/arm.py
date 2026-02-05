from fastapi import APIRouter, HTTPException
from backend.ra_controls import set_ra_mode as update_ra_mode
from backend.models_pydantic import RAModeRequest

router = APIRouter(prefix="/api/arm", tags=["arm"])

VALID_RA_MODES = ["disabled", "throttle", "ik-pos", "ik-vel"]


@router.post("/ra_mode/")
async def change_ra_mode(data: RAModeRequest):
    mode = data.mode
    if mode not in VALID_RA_MODES:
        raise HTTPException(
            status_code=400, detail=f"Invalid mode '{mode}'. Must be one of: {', '.join(VALID_RA_MODES)}"
        )

    await update_ra_mode(mode)
    return {"status": "success", "mode": mode}

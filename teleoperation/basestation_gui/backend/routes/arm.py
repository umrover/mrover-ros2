from fastapi import APIRouter, HTTPException
from backend.ra_controls import set_ra_mode as update_ra_mode
from backend.models_pydantic import RAModeRequest

router = APIRouter(prefix="/api/arm", tags=["arm"])

VALID_RA_MODES = ["disabled", "throttle", "ik-pos", "ik-vel"]
# TODO(stow): Add "stow" to VALID_RA_MODES once the stow mode is implemented in ra_controls.py.


@router.post("/ra_mode/")
async def change_ra_mode(data: RAModeRequest):
    mode = data.mode
    if mode not in VALID_RA_MODES:
        raise HTTPException(
            status_code=400, detail=f"Invalid mode '{mode}'. Must be one of: {', '.join(VALID_RA_MODES)}"
        )

    await update_ra_mode(mode)
    return {"status": "success", "mode": mode}


# TODO(stow): Add POST /stow/ endpoint. Should return stow target coordinates
# so the frontend can track progress.

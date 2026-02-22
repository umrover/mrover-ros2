from fastapi import APIRouter, HTTPException
from backend.database import get_recordings_db
from backend.models_pydantic import RecordingCreateRequest, RecordingWaypointRequest
from backend.managers.recording import get_recording_manager

router = APIRouter(prefix="/api/recordings", tags=["recordings"])


@router.get("/")
def get_recordings():
    conn = None
    try:
        conn = get_recordings_db()
        recordings = conn.execute('''
            SELECT r.id, r.name, r.is_drone, r.created_at, COUNT(w.id) as waypoint_count
            FROM recordings r
            LEFT JOIN recorded_waypoints w ON r.id = w.recording_id
            GROUP BY r.id
            ORDER BY r.created_at DESC
        ''').fetchall()
        return {'status': 'success', 'recordings': [dict(r) for r in recordings]}
    finally:
        if conn:
            conn.close()


@router.post("/create/")
def create_recording(data: RecordingCreateRequest):
    try:
        recording_manager = get_recording_manager()
        rec_id = recording_manager.start_recording(data.name, data.is_drone)
        return {'status': 'success', 'recording_id': rec_id}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to start recording: {str(e)}")


@router.post("/stop/")
def stop_recording():
    try:
        recording_manager = get_recording_manager()
        result = recording_manager.stop_recording()
        return {
            'status': 'success',
            'recording_id': result['recording_id'],
            'waypoint_count': result['waypoint_count']
        }
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to stop recording: {str(e)}")


@router.post("/{rec_id}/waypoints/")
def add_recording_waypoint(rec_id: int, data: RecordingWaypointRequest):
    conn = None
    try:
        conn = get_recordings_db()
        conn.execute('''
            INSERT INTO recorded_waypoints (recording_id, latitude, longitude, sequence)
            VALUES (?, ?, ?, ?)
        ''', (rec_id, data.lat, data.lon, data.sequence))
        conn.commit()
        return {'status': 'success'}
    finally:
        if conn:
            conn.close()


@router.get("/{rec_id}/waypoints/")
def get_recording_waypoints(rec_id: int):
    conn = None
    try:
        conn = get_recordings_db()
        waypoints = conn.execute('''
            SELECT id, latitude as lat, longitude as lon, timestamp, sequence
            FROM recorded_waypoints
            WHERE recording_id = ?
            ORDER BY sequence ASC
        ''', (rec_id,)).fetchall()
        return {'status': 'success', 'waypoints': [dict(w) for w in waypoints]}
    finally:
        if conn:
            conn.close()


@router.delete("/{rec_id}/")
def delete_recording(rec_id: int):
    conn = None
    try:
        conn = get_recordings_db()
        conn.execute('DELETE FROM recordings WHERE id = ?', (rec_id,))
        conn.commit()
        return {'status': 'success', 'deleted': rec_id}
    finally:
        if conn:
            conn.close()


@router.delete("/clear/")
def clear_recordings():
    conn = None
    try:
        conn = get_recordings_db()
        conn.execute('DELETE FROM recordings')
        conn.commit()
        return {'status': 'success'}
    finally:
        if conn:
            conn.close()

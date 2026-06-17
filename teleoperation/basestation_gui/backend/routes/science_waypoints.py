from fastapi import APIRouter, HTTPException
from backend.database import get_db_connection
from backend.models_pydantic import ScienceWaypoint
from backend.managers.recording import get_recording_manager

router = APIRouter(prefix="/api/science-waypoints", tags=["science-waypoints"])


@router.get("/gps-snapshot/")
def get_gps_snapshot():
    manager = get_recording_manager()
    return {
        'status': 'success',
        'lat': manager.rover_lat,
        'lon': manager.rover_lon,
        'altitude': manager.rover_alt,
    }


@router.get("/")
def get_science_waypoints():
    conn = None
    try:
        conn = get_db_connection()
        rows = conn.execute('SELECT * FROM science_waypoints ORDER BY id ASC').fetchall()
        results = []
        for w in rows:
            wd = dict(w)
            wd['lat'] = wd.pop('latitude')
            wd['lon'] = wd.pop('longitude')
            results.append(wd)
        return {'status': 'success', 'waypoints': results}
    finally:
        if conn:
            conn.close()


@router.post("/")
def create_science_waypoint(data: ScienceWaypoint):
    conn = None
    try:
        conn = get_db_connection()
        cursor = conn.execute(
            'INSERT INTO science_waypoints (name, latitude, longitude, altitude) VALUES (?, ?, ?, ?)',
            (data.name, data.lat, data.lon, data.altitude)
        )
        conn.commit()
        return {
            'status': 'success',
            'waypoint': {
                'id': cursor.lastrowid,
                'name': data.name,
                'lat': data.lat,
                'lon': data.lon,
                'altitude': data.altitude,
            }
        }
    finally:
        if conn:
            conn.close()


@router.delete("/clear/")
def clear_science_waypoints():
    conn = None
    try:
        conn = get_db_connection()
        conn.execute('DELETE FROM science_waypoints')
        conn.commit()
        return {'status': 'success'}
    finally:
        if conn:
            conn.close()


@router.post("/reset/")
def reset_science_waypoints():
    conn = None
    try:
        conn = get_db_connection()
        conn.execute('DROP TABLE IF EXISTS science_waypoints')
        conn.execute('''
            CREATE TABLE science_waypoints (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL,
                latitude REAL NOT NULL,
                longitude REAL NOT NULL,
                altitude REAL NOT NULL,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        conn.commit()
        return {'status': 'success'}
    finally:
        if conn:
            conn.close()


@router.delete("/{waypoint_id}/")
def delete_science_waypoint(waypoint_id: int):
    conn = None
    try:
        conn = get_db_connection()
        wp = conn.execute('SELECT id FROM science_waypoints WHERE id = ?', (waypoint_id,)).fetchone()
        if not wp:
            raise HTTPException(status_code=404, detail="Waypoint not found")
        conn.execute('DELETE FROM science_waypoints WHERE id = ?', (waypoint_id,))
        conn.commit()
        return {'status': 'success'}
    finally:
        if conn:
            conn.close()

from fastapi import APIRouter, HTTPException, status
from backend.database import get_db_connection
from backend.models_pydantic import BasicWaypointList, AutonWaypointList, RecordingCreateRequest, RecordingWaypointRequest

router = APIRouter(prefix="/api/waypoints", tags=["waypoints"])

# --- Basic Waypoints ---

@router.get("/basic/")
def get_basic_waypoints():
    conn = get_db_connection()
    waypoints = conn.execute('SELECT * FROM basic_waypoints').fetchall()
    conn.close()
    return {
        'status': 'success',
        'waypoints': [dict(w) for w in waypoints]
    }

@router.post("/basic/save/")
def save_basic_waypoints(data: BasicWaypointList):
    waypoints = data.waypoints
    
    conn = get_db_connection()
    conn.execute('DELETE FROM basic_waypoints')
    for w in waypoints:
        conn.execute(
            'INSERT INTO basic_waypoints (name, latitude, longitude, drone) VALUES (?, ?, ?, ?)',
            (w.name, w.lat, w.lon, w.drone)
        )
    conn.commit()
    conn.close()
    
    return {'status': 'success', 'count': len(waypoints)}

@router.delete("/basic/clear/")
def clear_basic_waypoints():
    conn = get_db_connection()
    conn.execute('DELETE FROM basic_waypoints')
    conn.commit()
    conn.close()
    return {'status': 'success', 'waypoints': []}

# --- Auton Waypoints (Store) ---

@router.get("/auton/")
def get_auton_waypoints():
    conn = get_db_connection()
    waypoints = conn.execute('SELECT * FROM auton_waypoints').fetchall()
    conn.close()
    
    results = []
    for w in waypoints:
        wd = dict(w)
        wd['lat'] = wd.pop('latitude')
        wd['lon'] = wd.pop('longitude')
        wd['enable_costmap'] = bool(wd['enable_costmap'])
        wd['deletable'] = bool(wd['deletable'])
        results.append(wd)
        
    return {'status': 'success', 'waypoints': results}

@router.post("/auton/save/")
def save_auton_waypoints(data: AutonWaypointList):
    waypoints = data.waypoints
    
    conn = get_db_connection()
    conn.execute('DELETE FROM auton_waypoints')
    
    for w in waypoints:
        conn.execute('''
            INSERT INTO auton_waypoints (name, tag_id, type, latitude, longitude, enable_costmap, deletable)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        ''', (
            w.name, 
            w.id, 
            w.type, 
            w.lat, 
            w.lon, 
            w.enable_costmap,
            w.deletable
        ))
    
    conn.commit()
    conn.close()
    return {'status': 'success'}

@router.delete("/auton/{waypoint_id}/")
def delete_auton_waypoint(waypoint_id: int):
    conn = get_db_connection()
    
    wp = conn.execute('SELECT deletable FROM auton_waypoints WHERE id = ?', (waypoint_id,)).fetchone()
    if not wp:
        conn.close()
        raise HTTPException(status_code=404, detail="Waypoint not found")
        
    if not wp['deletable']:
        conn.close()
        raise HTTPException(status_code=403, detail="This waypoint cannot be deleted")

    conn.execute('DELETE FROM auton_waypoints WHERE id = ?', (waypoint_id,))
    conn.commit()
    conn.close()
    return {'status': 'success', 'message': f'Waypoint {waypoint_id} deleted'}

# --- Current Auton Course (Active Route) ---

@router.get("/auton/current/")
def get_current_auton_course():
    conn = get_db_connection()
    course = conn.execute('SELECT * FROM current_auton_course ORDER BY sequence_order ASC').fetchall()
    conn.close()
    
    results = []
    for w in course:
        wd = dict(w)
        wd['lat'] = wd.pop('latitude')
        wd['lon'] = wd.pop('longitude')
        wd['enable_costmap'] = bool(wd['enable_costmap'])
        results.append(wd)

    return {'status': 'success', 'course': results}

@router.post("/auton/current/save/")
def save_current_auton_course(data: AutonWaypointList):
    course = data.waypoints
    
    conn = get_db_connection()
    conn.execute('DELETE FROM current_auton_course')
    
    for i, w in enumerate(course):
        conn.execute('''
            INSERT INTO current_auton_course (name, tag_id, type, latitude, longitude, enable_costmap, sequence_order)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        ''', (
            w.name,
            w.id,
            w.type,
            w.lat,
            w.lon,
            w.enable_costmap,
            i
        ))
        
    conn.commit()
    conn.close()
    return {'status': 'success'}

# --- Recordings ---

@router.get("/recordings/")
def get_recordings():
    conn = get_db_connection()
    recordings = conn.execute('''
        SELECT r.id, r.name, r.is_drone, r.created_at, COUNT(w.id) as waypoint_count
        FROM recordings r
        LEFT JOIN recorded_waypoints w ON r.id = w.recording_id
        GROUP BY r.id
        ORDER BY r.created_at DESC
    ''').fetchall()
    conn.close()
    return {'status': 'success', 'recordings': [dict(r) for r in recordings]}

@router.post("/recordings/create/")
def create_recording(data: RecordingCreateRequest):
    conn = get_db_connection()
    cur = conn.execute('INSERT INTO recordings (name, is_drone) VALUES (?, ?)', (data.name, data.is_drone))
    rec_id = cur.lastrowid
    conn.commit()
    conn.close()
    
    return {'status': 'success', 'recording_id': rec_id}

@router.post("/recordings/{rec_id}/waypoints/")
def add_recording_waypoint(rec_id: int, data: RecordingWaypointRequest):
    conn = get_db_connection()
    conn.execute('''
        INSERT INTO recorded_waypoints (recording_id, latitude, longitude, sequence)
        VALUES (?, ?, ?, ?)
    ''', (rec_id, data.lat, data.lon, data.sequence))
    conn.commit()
    conn.close()
    return {'status': 'success'}

@router.get("/recordings/{rec_id}/waypoints/")
def get_recording_waypoints(rec_id: int):
    conn = get_db_connection()
    waypoints = conn.execute('''
        SELECT id, latitude as lat, longitude as lon, timestamp, sequence
        FROM recorded_waypoints 
        WHERE recording_id = ?
        ORDER BY sequence ASC
    ''', (rec_id,)).fetchall()
    conn.close()
    return {'status': 'success', 'waypoints': [dict(w) for w in waypoints]}

@router.delete("/recordings/{rec_id}/")
def delete_recording(rec_id: int):
    conn = get_db_connection()
    conn.execute('DELETE FROM recordings WHERE id = ?', (rec_id,))
    conn.commit()
    conn.close()
    return {'status': 'success', 'deleted': rec_id}

@router.delete("/recordings/clear/")
def clear_recordings():
    conn = get_db_connection()
    conn.execute('DELETE FROM recordings')
    conn.commit()
    conn.close()
    return {'status': 'success'}

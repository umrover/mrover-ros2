from fastapi import APIRouter, HTTPException, status
from backend.database import get_db_connection
from backend.models_pydantic import BasicWaypointList, AutonWaypointList

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

from fastapi import APIRouter, HTTPException
from backend.database import get_db_connection
from backend.models_pydantic import BasicWaypointList, AutonWaypointList

router = APIRouter(prefix="/api/waypoints", tags=["waypoints"])


@router.get("/basic/")
def get_basic_waypoints():
    conn = None
    try:
        conn = get_db_connection()
        waypoints = conn.execute('SELECT * FROM basic_waypoints').fetchall()
        return {
            'status': 'success',
            'waypoints': [dict(w) for w in waypoints]
        }
    finally:
        if conn:
            conn.close()


@router.post("/basic/save/")
def save_basic_waypoints(data: BasicWaypointList):
    waypoints = data.waypoints
    conn = None
    try:
        conn = get_db_connection()
        conn.execute('DELETE FROM basic_waypoints')
        for w in waypoints:
            conn.execute(
                'INSERT INTO basic_waypoints (name, latitude, longitude, drone) VALUES (?, ?, ?, ?)',
                (w.name, w.lat, w.lon, w.drone)
            )
        conn.commit()
        return {'status': 'success', 'count': len(waypoints)}
    finally:
        if conn:
            conn.close()


@router.delete("/basic/clear/")
def clear_basic_waypoints():
    conn = None
    try:
        conn = get_db_connection()
        conn.execute('DELETE FROM basic_waypoints')
        conn.commit()
        return {'status': 'success', 'waypoints': []}
    finally:
        if conn:
            conn.close()


@router.get("/auton/")
def get_auton_waypoints():
    conn = None
    try:
        conn = get_db_connection()
        waypoints = conn.execute('SELECT * FROM auton_waypoints').fetchall()

        results = []
        for w in waypoints:
            wd = dict(w)
            wd['db_id'] = wd.pop('id')
            wd['lat'] = wd.pop('latitude')
            wd['lon'] = wd.pop('longitude')
            wd['enable_costmap'] = bool(wd['enable_costmap'])
            wd['deletable'] = bool(wd['deletable'])
            results.append(wd)

        return {'status': 'success', 'waypoints': results}
    finally:
        if conn:
            conn.close()


@router.post("/auton/save/")
def save_auton_waypoints(data: AutonWaypointList):
    waypoints = data.waypoints
    conn = None
    try:
        conn = get_db_connection()

        conn.execute('DELETE FROM auton_waypoints WHERE deletable = 1')
        conn.execute('DELETE FROM sqlite_sequence WHERE name = "auton_waypoints"')
        conn.execute('INSERT INTO sqlite_sequence (name, seq) VALUES ("auton_waypoints", 8)')

        for w in waypoints:
            if w.db_id is not None and 1 <= w.db_id <= 8:
                conn.execute('''
                    UPDATE auton_waypoints
                    SET name=?, tag_id=?, type=?, latitude=?, longitude=?, enable_costmap=?, coverage_radius=?
                    WHERE id=?
                ''', (w.name, w.tag_id, w.type, w.lat, w.lon, w.enable_costmap, w.coverage_radius, w.db_id))
            elif w.deletable:
                conn.execute('''
                    INSERT INTO auton_waypoints (name, tag_id, type, latitude, longitude, enable_costmap, coverage_radius, deletable)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                ''', (w.name, w.tag_id, w.type, w.lat, w.lon, w.enable_costmap, w.coverage_radius, w.deletable))

        conn.commit()
        return {'status': 'success'}
    finally:
        if conn:
            conn.close()


@router.delete("/auton/clear/")
def clear_auton_waypoints():
    """Clear all user-added auton waypoints (deletable=1), keep defaults."""
    conn = None
    try:
        conn = get_db_connection()
        conn.execute('DELETE FROM auton_waypoints WHERE deletable = 1')
        conn.execute('DELETE FROM current_auton_course')
        conn.commit()
        return {'status': 'success', 'message': 'User waypoints and current course cleared'}
    finally:
        if conn:
            conn.close()


@router.delete("/auton/clear/all/")
def clear_all_auton_waypoints():
    """Clear ALL auton waypoints including defaults. Use with caution."""
    conn = get_db_connection()
    conn.execute('DELETE FROM auton_waypoints')
    conn.execute('DELETE FROM current_auton_course')
    conn.execute('DELETE FROM sqlite_sequence WHERE name = "auton_waypoints"')
    conn.commit()
    conn.close()
    return {'status': 'success', 'message': 'All waypoints cleared'}

@router.delete("/auton/{waypoint_id}/")
def delete_auton_waypoint(waypoint_id: int):
    conn = None
    try:
        conn = get_db_connection()

        wp = conn.execute('SELECT deletable FROM auton_waypoints WHERE id = ?', (waypoint_id,)).fetchone()
        if not wp:
            raise HTTPException(status_code=404, detail="Waypoint not found")

        if not wp['deletable']:
            raise HTTPException(status_code=403, detail="This waypoint cannot be deleted")

        conn.execute('DELETE FROM auton_waypoints WHERE id = ?', (waypoint_id,))
        conn.commit()
        return {'status': 'success', 'message': f'Waypoint {waypoint_id} deleted'}
    finally:
        if conn:
            conn.close()


@router.get("/auton/current/")
def get_current_auton_course():
    conn = None
    try:
        conn = get_db_connection()
        course = conn.execute('SELECT * FROM current_auton_course ORDER BY sequence_order ASC').fetchall()

        results = []
        for w in course:
            wd = dict(w)
            del wd['id']
            wd['lat'] = wd.pop('latitude')
            wd['lon'] = wd.pop('longitude')
            wd['enable_costmap'] = bool(wd['enable_costmap'])
            del wd['sequence_order']
            results.append(wd)

        return {'status': 'success', 'course': results}
    finally:
        if conn:
            conn.close()


@router.post("/auton/current/save/")
def save_current_auton_course(data: AutonWaypointList):
    course = data.waypoints
    conn = None
    try:
        conn = get_db_connection()
        conn.execute('DELETE FROM current_auton_course')

        for i, w in enumerate(course):
            conn.execute('''
                INSERT INTO current_auton_course (name, tag_id, type, latitude, longitude, enable_costmap, coverage_radius, sequence_order)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            ''', (
                w.name,
                w.tag_id,
                w.type,
                w.lat,
                w.lon,
                w.enable_costmap,
                w.coverage_radius,
                i
            ))

        conn.commit()
        return {'status': 'success'}
    finally:
        if conn:
            conn.close()

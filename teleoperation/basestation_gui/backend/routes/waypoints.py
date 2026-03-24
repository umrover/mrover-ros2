from fastapi import APIRouter, HTTPException
from backend.database import get_db_connection
from backend.models_pydantic import BasicWaypoint, BasicWaypointList, AutonWaypointList, CreateAutonWaypoint, UpdateAutonWaypoint, UpdateBasicWaypoint

router = APIRouter(prefix="/api/waypoints", tags=["waypoints"])


@router.get("/basic/")
def get_basic_waypoints():
    conn = None
    try:
        conn = get_db_connection()
        waypoints = conn.execute('SELECT * FROM basic_waypoints').fetchall()
        results = []
        for w in waypoints:
            wd = dict(w)
            wd['db_id'] = wd.pop('id')
            wd['lat'] = wd.pop('latitude')
            wd['lon'] = wd.pop('longitude')
            wd['drone'] = bool(wd['drone'])
            results.append(wd)
        return {
            'status': 'success',
            'waypoints': results
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


@router.post("/basic/")
def create_basic_waypoint(data: BasicWaypoint):
    conn = None
    try:
        conn = get_db_connection()
        cursor = conn.execute(
            'INSERT INTO basic_waypoints (name, latitude, longitude, drone) VALUES (?, ?, ?, ?)',
            (data.name, data.lat, data.lon, data.drone)
        )
        conn.commit()
        db_id = cursor.lastrowid
        return {
            'status': 'success',
            'waypoint': {
                'db_id': db_id,
                'name': data.name,
                'lat': data.lat,
                'lon': data.lon,
                'drone': data.drone,
            }
        }
    finally:
        if conn:
            conn.close()


@router.patch("/basic/{waypoint_id}/")
def update_basic_waypoint(waypoint_id: int, data: UpdateBasicWaypoint):
    conn = None
    try:
        conn = get_db_connection()
        wp = conn.execute('SELECT * FROM basic_waypoints WHERE id = ?', (waypoint_id,)).fetchone()
        if not wp:
            raise HTTPException(status_code=404, detail="Waypoint not found")

        fields = data.model_dump(exclude_none=True)
        if not fields:
            raise HTTPException(status_code=400, detail="No fields to update")

        col_map = {'lat': 'latitude', 'lon': 'longitude'}
        allowed_cols = {'name', 'latitude', 'longitude', 'drone'}
        set_clauses = []
        values = []
        for key, val in fields.items():
            col = col_map.get(key, key)
            if col not in allowed_cols:
                raise HTTPException(status_code=400, detail=f"Invalid field: {key}")
            set_clauses.append(f'{col} = ?')
            values.append(val)

        values.append(waypoint_id)
        conn.execute(f'UPDATE basic_waypoints SET {", ".join(set_clauses)} WHERE id = ?', values)
        conn.commit()

        updated = conn.execute('SELECT * FROM basic_waypoints WHERE id = ?', (waypoint_id,)).fetchone()
        wd = dict(updated)
        wd['db_id'] = wd.pop('id')
        wd['lat'] = wd.pop('latitude')
        wd['lon'] = wd.pop('longitude')
        wd['drone'] = bool(wd['drone'])

        return {'status': 'success', 'waypoint': wd}
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


@router.delete("/basic/{waypoint_id}/")
def delete_basic_waypoint(waypoint_id: int):
    conn = None
    try:
        conn = get_db_connection()
        wp = conn.execute('SELECT * FROM basic_waypoints WHERE id = ?', (waypoint_id,)).fetchone()
        if not wp:
            raise HTTPException(status_code=404, detail="Waypoint not found")

        conn.execute('DELETE FROM basic_waypoints WHERE id = ?', (waypoint_id,))
        conn.commit()
        return {'status': 'success', 'message': f'Waypoint {waypoint_id} deleted'}
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


@router.post("/auton/")
def create_auton_waypoint(data: CreateAutonWaypoint):
    conn = None
    try:
        conn = get_db_connection()
        cursor = conn.execute('''
            INSERT INTO auton_waypoints (name, tag_id, type, latitude, longitude, enable_costmap, deletable)
            VALUES (?, ?, ?, ?, ?, ?, 1)
        ''', (data.name, data.tag_id, data.type, data.lat, data.lon, data.enable_costmap))
        conn.commit()
        db_id = cursor.lastrowid

        return {
            'status': 'success',
            'waypoint': {
                'db_id': db_id,
                'name': data.name,
                'tag_id': data.tag_id,
                'type': data.type,
                'lat': data.lat,
                'lon': data.lon,
                'enable_costmap': data.enable_costmap,
                'deletable': True,
            }
        }
    finally:
        if conn:
            conn.close()


@router.patch("/auton/{waypoint_id}/")
def update_auton_waypoint(waypoint_id: int, data: UpdateAutonWaypoint):
    conn = None
    try:
        conn = get_db_connection()
        wp = conn.execute('SELECT * FROM auton_waypoints WHERE id = ?', (waypoint_id,)).fetchone()
        if not wp:
            raise HTTPException(status_code=404, detail="Waypoint not found")

        fields = data.model_dump(exclude_none=True)
        if not fields:
            raise HTTPException(status_code=400, detail="No fields to update")

        col_map = {'lat': 'latitude', 'lon': 'longitude'}
        allowed_cols = {'name', 'tag_id', 'type', 'latitude', 'longitude', 'enable_costmap'}
        set_clauses = []
        values = []
        for key, val in fields.items():
            col = col_map.get(key, key)
            if col not in allowed_cols:
                raise HTTPException(status_code=400, detail=f"Invalid field: {key}")
            set_clauses.append(f'{col} = ?')
            values.append(val)

        values.append(waypoint_id)
        conn.execute(f'UPDATE auton_waypoints SET {", ".join(set_clauses)} WHERE id = ?', values)
        conn.commit()

        updated = conn.execute('SELECT * FROM auton_waypoints WHERE id = ?', (waypoint_id,)).fetchone()
        wd = dict(updated)
        wd['db_id'] = wd.pop('id')
        wd['lat'] = wd.pop('latitude')
        wd['lon'] = wd.pop('longitude')
        wd['enable_costmap'] = bool(wd['enable_costmap'])
        wd['deletable'] = bool(wd['deletable'])

        return {'status': 'success', 'waypoint': wd}
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
    conn = None
    try:
        conn = get_db_connection()
        conn.execute('DELETE FROM auton_waypoints')
        conn.execute('DELETE FROM current_auton_course')
        conn.execute('DELETE FROM sqlite_sequence WHERE name = "auton_waypoints"')
        conn.commit()
        return {'status': 'success', 'message': 'All waypoints cleared'}
    finally:
        if conn:
            conn.close()

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
                INSERT INTO current_auton_course (name, tag_id, type, latitude, longitude, enable_costmap, sequence_order)
                VALUES (?, ?, ?, ?, ?, ?, ?)
            ''', (
                w.name,
                w.tag_id,
                w.type,
                w.lat,
                w.lon,
                w.enable_costmap,
                i
            ))

        conn.commit()
        return {'status': 'success'}
    finally:
        if conn:
            conn.close()

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


def fetch_store_row(conn, waypoint_id: int) -> dict:
    row = conn.execute('SELECT * FROM auton_waypoints WHERE id = ?', (waypoint_id,)).fetchone()
    if not row:
        raise HTTPException(status_code=404, detail="Waypoint not found")
    wd = dict(row)
    wd['db_id'] = wd.pop('id')
    wd['lat'] = wd.pop('latitude')
    wd['lon'] = wd.pop('longitude')
    wd['deletable'] = bool(wd['deletable'])
    return wd


@router.get("/auton/store/")
def get_store():
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
            wd['deletable'] = bool(wd['deletable'])
            results.append(wd)
        return {'status': 'success', 'waypoints': results}
    finally:
        if conn:
            conn.close()


@router.post("/auton/store/")
def add_to_store(data: CreateAutonWaypoint):
    conn = None
    try:
        conn = get_db_connection()
        cursor = conn.execute('''
            INSERT INTO auton_waypoints (name, tag_id, type, latitude, longitude, deletable)
            VALUES (?, ?, ?, ?, ?, 1)
        ''', (data.name, data.tag_id, data.type, data.lat, data.lon))
        conn.commit()
        return {
            'status': 'success',
            'waypoint': fetch_store_row(conn, cursor.lastrowid)
        }
    finally:
        if conn:
            conn.close()


@router.patch("/auton/store/{waypoint_id}/")
def update_store(waypoint_id: int, data: UpdateAutonWaypoint):
    conn = None
    try:
        conn = get_db_connection()
        wp = conn.execute('SELECT * FROM auton_waypoints WHERE id = ?', (waypoint_id,)).fetchone()
        if not wp:
            raise HTTPException(status_code=404, detail="Waypoint not found")

        fields = data.model_dump(exclude_unset=True)
        if not fields:
            raise HTTPException(status_code=400, detail="No fields to update")

        if not wp['deletable']:
            immutable = {'name', 'type'} & fields.keys()
            if immutable:
                raise HTTPException(
                    status_code=403,
                    detail=f"Cannot modify {', '.join(immutable)} on permanent waypoints"
                )

        col_map = {'lat': 'latitude', 'lon': 'longitude'}
        allowed_cols = {'name', 'tag_id', 'type', 'latitude', 'longitude'}
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

        return {'status': 'success', 'waypoint': fetch_store_row(conn, waypoint_id)}
    finally:
        if conn:
            conn.close()


@router.delete("/auton/store/{waypoint_id}/")
def remove_from_store(waypoint_id: int):
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


@router.delete("/auton/store/")
def reset_store():
    from backend.database import init_waypoints_db
    conn = None
    try:
        conn = get_db_connection()
        conn.execute('DROP TABLE IF EXISTS auton_waypoints')
        conn.execute('DROP TABLE IF EXISTS current_auton_course')
        conn.execute("DELETE FROM sqlite_sequence WHERE name IN ('auton_waypoints', 'current_auton_course')")
        conn.commit()
        conn.close()
        conn = None
        init_waypoints_db()
        return {'status': 'success', 'message': 'Tables recreated with defaults'}
    finally:
        if conn:
            conn.close()


@router.get("/auton/execution/")
def get_execution():
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
            del wd['sequence_order']
            results.append(wd)
        return {'status': 'success', 'course': results}
    finally:
        if conn:
            conn.close()


@router.post("/auton/execution/save/")
def save_execution(data: AutonWaypointList):
    conn = None
    try:
        conn = get_db_connection()
        conn.execute('DELETE FROM current_auton_course')
        for i, w in enumerate(data.waypoints):
            conn.execute('''
                INSERT INTO current_auton_course (name, tag_id, type, latitude, longitude, sequence_order)
                VALUES (?, ?, ?, ?, ?, ?)
            ''', (w.name, w.tag_id, w.type, w.lat, w.lon, i))
        conn.commit()
        return {'status': 'success'}
    finally:
        if conn:
            conn.close()

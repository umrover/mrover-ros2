from backend.models import BasicWaypoint, CurrentBasicWaypoints, AutonWaypoint, CurrentAutonWaypoints, WaypointRecording, RecordedWaypoint

def save_basic_waypoint_list(waypoints: list[dict]) -> None:
    BasicWaypoint.objects.all().delete()
    BasicWaypoint.objects.bulk_create(
        [BasicWaypoint(drone=w["drone"], latitude=w["lat"], longitude=w["lon"], name=w["name"]) for w in waypoints]
    )

def get_basic_waypoint_list() -> None:
    return [
        {"name": w.name, "drone": w.drone, "lat": w.latitude, "lon": w.longitude}
        for w in BasicWaypoint.objects.all()
    ]

def save_auton_waypoint_list(waypoints: list[dict]) -> None:
    AutonWaypoint.objects.all().delete()
    AutonWaypoint.objects.bulk_create(
        [
            AutonWaypoint(
                tag_id=w["id"],
                type=w["type"],
                latitude=w["lat"],
                longitude=w["lon"],
                name=w["name"],
                enable_costmap=w.get("enable_costmap", True),
            )
            for w in waypoints
        ]
    )

def get_auton_waypoint_list() -> None:
    return [
        {"name": w.name, "id": w.tag_id, "lat": w.latitude, "lon": w.longitude, "type": w.type, "enable_costmap": w.enable_costmap}
        for w in AutonWaypoint.objects.all()
    ]

def save_current_auton_course(waypoints: list[dict]) -> None:
    CurrentAutonWaypoints.objects.all().delete()
    CurrentAutonWaypoints.objects.bulk_create(
        [
            CurrentAutonWaypoints(
                tag_id=w["id"],
                type=w["type"],
                latitude=w["lat"],
                longitude=w["lon"],
                name=w["name"],
                enable_costmap=w['enable_costmap']
            )
            for w in waypoints
        ]
    )
    

def get_current_auton_course() -> None:
    return [
        {"name": w.name, "id": w.tag_id, "lat": w.latitude, "lon": w.longitude, "type": w.type, "enable_costmap": w.enable_costmap}
        for w in CurrentAutonWaypoints.objects.all()
    ]


def save_current_basic_course(waypoints: list[dict]) -> None:
    CurrentBasicWaypoints.objects.all().delete()
    CurrentBasicWaypoints.objects.bulk_create(
        [
            CurrentBasicWaypoints(
                drone=w["drone"], latitude=w["lat"], longitude=w["lon"], name=w["name"]
            )
            for w in waypoints
        ]
    )

def get_current_basic_course() -> None:
    return [
        w.id
        for w in CurrentBasicWaypoints.objects.all()
    ]

def delete_auton_waypoint_from_course(waypoint: object) -> None:
    CurrentAutonWaypoints.objects.filter(
    tag_id=waypoint["id"],
    type=waypoint["type"],
    name=waypoint["name"],
    latitude=waypoint["lat"],
    longitude=waypoint["lon"]
    ).delete()


def create_recording(name: str, is_drone: bool) -> int:
    recording = WaypointRecording.objects.create(name=name, is_drone=is_drone)
    return recording.id


def add_waypoint_to_recording(recording_id: int, latitude: float, longitude: float, sequence: int) -> None:
    recording = WaypointRecording.objects.get(id=recording_id)
    RecordedWaypoint.objects.create(
        recording=recording,
        latitude=latitude,
        longitude=longitude,
        sequence=sequence
    )


def get_all_recordings() -> list[dict]:
    recordings = WaypointRecording.objects.all().order_by('-created_at')
    return [
        {
            "id": r.id,
            "name": r.name,
            "is_drone": r.is_drone,
            "created_at": r.created_at.isoformat(),
            "waypoint_count": r.waypoints.count()
        }
        for r in recordings
    ]


def get_recording_waypoints(recording_id: int) -> list[dict]:
    waypoints = RecordedWaypoint.objects.filter(recording_id=recording_id).order_by('sequence')
    return [
        {
            "id": w.id,
            "lat": w.latitude,
            "lon": w.longitude,
            "timestamp": w.timestamp.isoformat(),
            "sequence": w.sequence
        }
        for w in waypoints
    ]


def delete_recording(recording_id: int) -> None:
    WaypointRecording.objects.filter(id=recording_id).delete()


def clear_all_basic_waypoints() -> None:
    BasicWaypoint.objects.all().delete()


def clear_all_recordings() -> None:
    from django.db import connection
    WaypointRecording.objects.all().delete()
    with connection.cursor() as cursor:
        cursor.execute("DELETE FROM sqlite_sequence WHERE name='backend_waypointrecording'")
        cursor.execute("DELETE FROM sqlite_sequence WHERE name='backend_recordedwaypoint'")

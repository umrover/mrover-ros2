from backend.models import BasicWaypoint, CurrentBasicWaypoints, AutonWaypoint, CurrentAutonWaypoints

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
            )
            for w in waypoints
        ]
    )

def get_auton_waypoint_list() -> None:
    return [
        {"name": w.name, "id": w.tag_id, "lat": w.latitude, "lon": w.longitude, "type": w.type}
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

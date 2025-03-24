from backend.models import BasicWaypoint, AutonWaypoint

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

from typing import Optional
from backend.consumers.ros_manager import get_node
from backend.models import WaypointRecording, RecordedWaypoint
from sensor_msgs.msg import NavSatFix
from rclpy.qos import qos_profile_sensor_data

RECORDING_RATE_HZ = 5

class RecordingManager:
    def __init__(self):
        self.node = get_node()
        self.is_recording = False
        self.current_recording_id: Optional[int] = None
        self.recording_sequence = 0
        self.is_drone_recording = False
        self.timer = None

        self.rover_lat = 0.0
        self.rover_lon = 0.0
        self.drone_lat = 0.0
        self.drone_lon = 0.0

        self.rover_gps_sub = self.node.create_subscription(
            NavSatFix,
            "/gps/fix",
            self._handle_rover_gps,
            qos_profile=qos_profile_sensor_data
        )

        self.drone_gps_sub = self.node.create_subscription(
            NavSatFix,
            "/drone_odometry",
            self._handle_drone_gps,
            qos_profile=qos_profile_sensor_data
        )

    def _handle_rover_gps(self, msg: NavSatFix):
        self.rover_lat = msg.latitude
        self.rover_lon = msg.longitude

    def _handle_drone_gps(self, msg: NavSatFix):
        self.drone_lat = msg.latitude
        self.drone_lon = msg.longitude

    def _recording_callback(self):
        if not self.is_recording or self.current_recording_id is None:
            return

        lat = self.drone_lat if self.is_drone_recording else self.rover_lat
        lon = self.drone_lon if self.is_drone_recording else self.rover_lon

        if lat == 0.0 and lon == 0.0:
            return

        try:
            RecordedWaypoint.objects.create(
                recording_id=self.current_recording_id,
                latitude=lat,
                longitude=lon,
                sequence=self.recording_sequence
            )
            self.recording_sequence += 1
        except Exception as e:
            print(f"Failed to save waypoint: {e}")

    def start_recording(self, name: str, is_drone: bool) -> int:
        if self.is_recording:
            raise ValueError("Recording already in progress")

        recording = WaypointRecording.objects.create(
            name=name,
            is_drone=is_drone
        )

        self.current_recording_id = recording.id
        self.recording_sequence = 0
        self.is_drone_recording = is_drone
        self.is_recording = True

        self.timer = self.node.create_timer(
            1.0 / RECORDING_RATE_HZ,
            self._recording_callback
        )

        print(f"Started recording: {name} (ID: {recording.id}) at {RECORDING_RATE_HZ}Hz")

        return recording.id

    def stop_recording(self) -> dict:
        if not self.is_recording:
            raise ValueError("No recording in progress")

        waypoint_count = self.recording_sequence
        recording_id = self.current_recording_id

        if self.timer:
            self.node.destroy_timer(self.timer)
            self.timer = None

        self.is_recording = False
        self.current_recording_id = None
        self.recording_sequence = 0
        self.is_drone_recording = False

        print(f"Stopped recording ID: {recording_id} with {waypoint_count} waypoints")

        return {
            "recording_id": recording_id,
            "waypoint_count": waypoint_count
        }

    def get_status(self) -> dict:
        return {
            "is_recording": self.is_recording,
            "recording_id": self.current_recording_id,
            "waypoint_count": self.recording_sequence,
            "is_drone": self.is_drone_recording
        }

_recording_manager = None

def get_recording_manager() -> RecordingManager:
    global _recording_manager
    if _recording_manager is None:
        _recording_manager = RecordingManager()
    return _recording_manager

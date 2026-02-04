from django.urls import path

from backend.consumers.arm_consumer import ArmConsumer
from backend.consumers.auton_consumer import AutonConsumer
from backend.consumers.drive_consumer import DriveConsumer
from backend.consumers.mast_consumer import MastConsumer
from backend.consumers.nav_consumer import NavConsumer
from backend.consumers.science_consumer import ScienceConsumer
from backend.consumers.waypoints_consumer import WaypointsConsumer

# FOR CONSUMER TOPICS/SERVICES LOOKUP, SEE https://github.com/umrover/mrover-ros2/wiki/Teleop-Consumers-Lookup

websocket_urlpatterns = [
    path("ws/arm", ArmConsumer.as_asgi()),
    path("ws/auton", AutonConsumer.as_asgi()),
    path("ws/drive", DriveConsumer.as_asgi()),
    path("ws/mast", MastConsumer.as_asgi()),
    path("ws/nav", NavConsumer.as_asgi()),
    path("ws/science", ScienceConsumer.as_asgi()),
    path("ws/waypoints", WaypointsConsumer.as_asgi()),
]

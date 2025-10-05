"""
ASGI config for basestation_gui project.

It exposes the ASGI callable as a module-level variable named ``application``.

For more information on this file, see
https://docs.djangoproject.com/en/4.2/howto/deployment/asgi/
"""
import os
import asyncio
from channels.auth import AuthMiddlewareStack
from channels.routing import ProtocolTypeRouter, URLRouter
from django.core.asgi import get_asgi_application
import basestation_gui.routing

os.environ.setdefault("DJANGO_SETTINGS_MODULE", "basestation_gui.settings")

django_asgi_app = get_asgi_application()

# Wrapper to initialize ROS on ASGI server startup
class ROSLifespanWrapper:
    def __init__(self, app):
        self.app = app
        self.ros_initialized = False

    async def __call__(self, scope, receive, send):
        if scope["type"] == "lifespan":
            while True:
                message = await receive()
                if message["type"] == "lifespan.startup":
                    # Initialize ROS when ASGI server starts
                    from backend.consumers.ros_manager import start_ros_executor
                    await start_ros_executor()
                    self.ros_initialized = True
                    await send({"type": "lifespan.startup.complete"})
                elif message["type"] == "lifespan.shutdown":
                    await send({"type": "lifespan.shutdown.complete"})
                    return
        else:
            await self.app(scope, receive, send)

application = ROSLifespanWrapper(
    ProtocolTypeRouter(
        {
            "http": django_asgi_app,
            "websocket": AuthMiddlewareStack(
                URLRouter(basestation_gui.routing.websocket_urlpatterns)
            ),
        }
    )
)

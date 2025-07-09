"""
URL configuration for basestation_gui project.

The `urlpatterns` list routes URLs to views. For more information please see:
    https://docs.djangoproject.com/en/4.2/topics/http/urls/
Examples:
Function views
    1. Add an import:  from my_app import views
    2. Add a URL to urlpatterns:  path('', views.home, name='home')
Class-based views
    1. Add an import:  from other_app.views import Home
    2. Add a URL to urlpatterns:  path('', Home.as_view(), name='home')
Including another URLconf
    1. Import the include() function: from django.urls import include, path
    2. Add a URL to urlpatterns:  path('blog/', include('blog.urls'))
"""

from django.contrib import admin
from django.urls import path, include
from django.http import HttpResponse
from django.shortcuts import render

from backend.consumers.arm_consumer import ArmConsumer
from backend.consumers.auton_consumer import AutonConsumer
from backend.consumers.drive_consumer import DriveConsumer
from backend.consumers.nav_consumer import NavConsumer
from backend.consumers.science_consumer import ScienceConsumer
from backend.consumers.waypoints_consumer import WaypointsConsumer

urlpatterns = [
    path("admin/", admin.site.urls),
]

websocket_urlpatterns = [
    path("ws/arm", ArmConsumer.as_asgi()),
    path("ws/auton", AutonConsumer.as_asgi()),
    path("ws/drive", DriveConsumer.as_asgi()),
    path("ws/nav", NavConsumer.as_asgi()),
    path("ws/science", ScienceConsumer.as_asgi()),
    path("ws/waypoints", WaypointsConsumer.as_asgi()),
]

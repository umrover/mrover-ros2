from django.apps import AppConfig
from django.db.models.signals import post_migrate
from django.dispatch import receiver
from django.db import OperationalError

class BackendConfig(AppConfig):
    default_auto_field = "django.db.models.BigAutoField"
    name = "backend"

    def ready(self):
        # Connect post_migrate signal to a handler method
        # after migrations are applied, (ensuring that the db is ready) we will clear the current auton course
        post_migrate.connect(clear_current_auton_waypoints, sender=self)

# function to clear current auton course, basically what you see in waypoints.py
def clear_current_auton_waypoints(sender, **kwargs):
    from .models import CurrentAutonWaypoints
    try:
        # Clear the table when the migrations are done
        CurrentAutonWaypoints.objects.all().delete()
        print("Cleared CurrentAutonWaypoints on startup after migrations")
    except OperationalError:
        print("Skipped clearing CurrentAutonWaypoints (DB not ready)")

from django.db import models


# Create your models here.
class AutonWaypoint(models.Model):
    id = models.AutoField(unique=True, primary_key=True)
    tag_id = models.IntegerField()
    # Type based on WaypointType.msg
    type = models.IntegerField()
    created_at = models.DateTimeField(auto_now_add=True)
    latitude = models.FloatField()
    longitude = models.FloatField()
    name = models.CharField(max_length=100)
    enable_costmap = models.BooleanField(default=True)


class BasicWaypoint(models.Model):
    id = models.AutoField(unique=True, primary_key=True)
    drone = models.BooleanField()
    created_at = models.DateTimeField(auto_now_add=True)
    latitude = models.FloatField()
    longitude = models.FloatField()
    name = models.CharField(max_length=100)


class CurrentBasicWaypoints(models.Model):
    id = models.AutoField(unique=True, primary_key=True)
    drone = models.BooleanField()
    latitude = models.FloatField()
    longitude = models.FloatField()
    name = models.CharField(max_length=100)


class CurrentAutonWaypoints(models.Model):
    id = models.AutoField(unique=True, primary_key=True)
    tag_id = models.IntegerField()
    type = models.IntegerField()
    latitude = models.FloatField()
    longitude = models.FloatField()
    name = models.CharField(max_length=100)
    enable_costmap = models.BooleanField(default=False)


class WaypointRecording(models.Model):
    id = models.AutoField(unique=True, primary_key=True)
    name = models.CharField(max_length=100)
    is_drone = models.BooleanField(default=False)
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)


class RecordedWaypoint(models.Model):
    id = models.AutoField(unique=True, primary_key=True)
    recording = models.ForeignKey(WaypointRecording, on_delete=models.CASCADE, related_name='waypoints')
    latitude = models.FloatField()
    longitude = models.FloatField()
    timestamp = models.DateTimeField(auto_now_add=True)
    sequence = models.IntegerField()
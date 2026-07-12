import pytest
import numpy as np
from navigation.trajectory import Trajectory

def test_get_current_point():
    points = np.array([[1, 2], [3, 4]])
    traj = Trajectory(points)
    assert (traj.get_current_point() == points[0]).all()

def test_increment_point():
    points = np.array([[1, 2], [3, 4]])
    traj = Trajectory(points)
    assert not traj.increment_point()
    assert (traj.get_current_point() == points[1]).all()
    assert traj.increment_point()
    # increment again and make sure it doesn't break anything
    assert traj.increment_point()

def test_decrement_point():
    points = np.array([[1, 2], [3, 4]])
    traj = Trajectory(points)
    # already at the start so this should do nothing
    assert traj.decerement_point()
    assert (traj.get_current_point() == points[0]).all()

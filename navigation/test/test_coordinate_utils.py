import pytest
from navigation.coordinate_utils import d_calc

def test_d_calc():
    assert d_calc((0, 0), (3, 4)) == 5

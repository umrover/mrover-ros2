import pytest
from api.conftest import url

pytestmark = pytest.mark.api


def _get_basic(api):
    resp = api.get(url(api, '/api/waypoints/basic/'))
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'success'
    return data['waypoints']


def _save_basic(api, waypoints):
    resp = api.post(url(api, '/api/waypoints/basic/save/'), json={'waypoints': waypoints})
    assert resp.status_code == 200
    return resp.json()


class TestBasicWaypoints:
    def test_empty_initially(self, api, clean_basic_waypoints):
        wps = _get_basic(api)
        assert wps == []

    def test_save_and_get(self, api, clean_basic_waypoints):
        wps = [
            {'name': 'A', 'lat': 42.0, 'lon': -83.0, 'drone': False},
            {'name': 'B', 'lat': 42.1, 'lon': -83.1, 'drone': False},
        ]
        _save_basic(api, wps)
        result = _get_basic(api)
        assert len(result) == 2

    def test_save_replaces_all(self, api, clean_basic_waypoints):
        _save_basic(api, [{'name': 'A', 'lat': 42.0, 'lon': -83.0, 'drone': False}])
        _save_basic(api, [{'name': 'B', 'lat': 42.1, 'lon': -83.1, 'drone': False}])
        result = _get_basic(api)
        assert len(result) == 1
        assert result[0]['name'] == 'B'

    def test_drone_flag_persists(self, api, clean_basic_waypoints):
        _save_basic(api, [{'name': 'D', 'lat': 42.0, 'lon': -83.0, 'drone': True}])
        result = _get_basic(api)
        assert result[0]['drone'] == 1 or result[0]['drone'] is True

    def test_clear(self, api, clean_basic_waypoints):
        _save_basic(api, [{'name': 'X', 'lat': 0, 'lon': 0, 'drone': False}])
        api.delete(url(api, '/api/waypoints/basic/clear/'))
        result = _get_basic(api)
        assert result == []

    def test_fields_present(self, api, clean_basic_waypoints):
        _save_basic(api, [{'name': 'F', 'lat': 42.5, 'lon': -83.5, 'drone': False}])
        result = _get_basic(api)
        wp = result[0]
        assert 'name' in wp
        assert 'latitude' in wp or 'lat' in wp
        assert 'longitude' in wp or 'lon' in wp
        assert 'drone' in wp

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


def _create_basic(api, **overrides):
    payload = {
        'name': 'Test Basic',
        'lat': 42.123,
        'lon': -83.456,
        'drone': False,
    }
    payload.update(overrides)
    resp = api.post(url(api, '/api/waypoints/basic/'), json=payload)
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'success'
    return data['waypoint']


class TestBasicWaypointsBulk:
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

    def test_clear(self, api, clean_basic_waypoints):
        _save_basic(api, [{'name': 'X', 'lat': 0, 'lon': 0, 'drone': False}])
        api.delete(url(api, '/api/waypoints/basic/clear/'))
        result = _get_basic(api)
        assert result == []


class TestBasicWaypointGet:
    def test_get_returns_db_id(self, api, clean_basic_waypoints):
        _create_basic(api)
        waypoints = _get_basic(api)
        assert len(waypoints) >= 1
        assert 'db_id' in waypoints[0]

    def test_get_returns_lat_lon(self, api, clean_basic_waypoints):
        _create_basic(api, lat=10.0, lon=20.0)
        waypoints = _get_basic(api)
        wp = waypoints[-1]
        assert 'lat' in wp
        assert 'lon' in wp
        assert abs(wp['lat'] - 10.0) < 0.001
        assert abs(wp['lon'] - 20.0) < 0.001

    def test_get_does_not_return_old_column_names(self, api, clean_basic_waypoints):
        _create_basic(api)
        waypoints = _get_basic(api)
        wp = waypoints[-1]
        assert 'id' not in wp
        assert 'latitude' not in wp
        assert 'longitude' not in wp

    def test_drone_flag_is_bool(self, api, clean_basic_waypoints):
        _create_basic(api, drone=True)
        waypoints = _get_basic(api)
        wp = waypoints[-1]
        assert wp['drone'] is True


class TestBasicWaypointCreate:
    def test_create_returns_db_id(self, api, clean_basic_waypoints):
        wp = _create_basic(api)
        assert 'db_id' in wp
        assert isinstance(wp['db_id'], int)

    def test_create_persists(self, api, clean_basic_waypoints):
        wp = _create_basic(api, name='persist-test')
        waypoints = _get_basic(api)
        assert any(w['db_id'] == wp['db_id'] for w in waypoints)

    def test_create_preserves_fields(self, api, clean_basic_waypoints):
        wp = _create_basic(api, name='field-test', lat=10.0, lon=20.0, drone=True)
        assert wp['name'] == 'field-test'
        assert abs(wp['lat'] - 10.0) < 0.001
        assert abs(wp['lon'] - 20.0) < 0.001
        assert wp['drone'] is True


class TestBasicWaypointUpdate:
    def test_partial_update_name(self, api, clean_basic_waypoints):
        wp = _create_basic(api, name='original')
        resp = api.patch(url(api, f'/api/waypoints/basic/{wp["db_id"]}/'), json={'name': 'updated'})
        assert resp.status_code == 200
        data = resp.json()
        assert data['waypoint']['name'] == 'updated'

    def test_partial_update_coords(self, api, clean_basic_waypoints):
        wp = _create_basic(api)
        resp = api.patch(url(api, f'/api/waypoints/basic/{wp["db_id"]}/'), json={'lat': 50.0, 'lon': -100.0})
        assert resp.status_code == 200
        data = resp.json()
        assert abs(data['waypoint']['lat'] - 50.0) < 0.001
        assert abs(data['waypoint']['lon'] - (-100.0)) < 0.001

    def test_update_nonexistent_returns_404(self, api, clean_basic_waypoints):
        resp = api.patch(url(api, '/api/waypoints/basic/9999/'), json={'name': 'nope'})
        assert resp.status_code == 404

    def test_update_empty_body_returns_400(self, api, clean_basic_waypoints):
        wp = _create_basic(api)
        resp = api.patch(url(api, f'/api/waypoints/basic/{wp["db_id"]}/'), json={})
        assert resp.status_code == 400

    def test_update_preserves_unchanged_fields(self, api, clean_basic_waypoints):
        wp = _create_basic(api, name='keep', lat=1.0, lon=2.0, drone=True)
        api.patch(url(api, f'/api/waypoints/basic/{wp["db_id"]}/'), json={'name': 'renamed'})
        waypoints = _get_basic(api)
        updated = next(w for w in waypoints if w['db_id'] == wp['db_id'])
        assert updated['name'] == 'renamed'
        assert abs(updated['lat'] - 1.0) < 0.001
        assert updated['drone'] is True


class TestBasicWaypointDelete:
    def test_delete_single(self, api, clean_basic_waypoints):
        wp = _create_basic(api)
        db_id = wp['db_id']
        resp = api.delete(url(api, f'/api/waypoints/basic/{db_id}/'))
        assert resp.status_code == 200
        waypoints = _get_basic(api)
        assert not any(w['db_id'] == db_id for w in waypoints)

    def test_delete_nonexistent_returns_404(self, api, clean_basic_waypoints):
        resp = api.delete(url(api, '/api/waypoints/basic/9999/'))
        assert resp.status_code == 404

    def test_clear_removes_all(self, api, clean_basic_waypoints):
        _create_basic(api, name='a')
        _create_basic(api, name='b')
        api.delete(url(api, '/api/waypoints/basic/clear/'))
        waypoints = _get_basic(api)
        assert len(waypoints) == 0

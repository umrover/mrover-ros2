import pytest
from api.conftest import url, seed_auton_defaults

pytestmark = pytest.mark.api

DEFAULT_NAMES = [
    'No Search 1', 'No Search 2',
    'Post 1', 'Post 2', 'Post 3',
    'Mallet', 'Water Bottle', 'Rock Pick',
]


def _get_waypoints(api):
    resp = api.get(url(api, '/api/waypoints/auton/'))
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'success'
    return data['waypoints']


def _create_waypoint(api, **overrides):
    payload = {
        'name': 'Test WP',
        'tag_id': 42,
        'type': 1,
        'lat': 42.123,
        'lon': -83.456,
        'enable_costmap': True,
        'coverage_radius': 0.0,
    }
    payload.update(overrides)
    resp = api.post(url(api, '/api/waypoints/auton/'), json=payload)
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'success'
    return data['waypoint']


class TestAutonWaypointDefaults:
    def test_get_default_waypoints_returns_8(self, api):
        waypoints = _get_waypoints(api)
        defaults = [w for w in waypoints if not w['deletable']]
        assert len(defaults) == 8

    def test_default_tag_ids_are_0_through_7(self, api):
        waypoints = _get_waypoints(api)
        defaults = sorted(
            [w for w in waypoints if not w['deletable']],
            key=lambda w: w['tag_id'],
        )
        tag_ids = [w['tag_id'] for w in defaults]
        assert tag_ids == list(range(8))

    def test_default_names_match(self, api):
        waypoints = _get_waypoints(api)
        defaults = sorted(
            [w for w in waypoints if not w['deletable']],
            key=lambda w: w['tag_id'],
        )
        names = [w['name'] for w in defaults]
        assert names == DEFAULT_NAMES

    def test_defaults_not_deletable(self, api):
        waypoints = _get_waypoints(api)
        defaults = [w for w in waypoints if not w['deletable']]
        for w in defaults:
            assert w['deletable'] is False


class TestAutonWaypointCreate:
    def test_create_returns_db_id(self, api, clean_auton_waypoints):
        wp = _create_waypoint(api)
        assert 'db_id' in wp
        assert wp['db_id'] > 8

    def test_create_persists(self, api, clean_auton_waypoints):
        wp = _create_waypoint(api, name='persist-test')
        waypoints = _get_waypoints(api)
        assert any(w['db_id'] == wp['db_id'] for w in waypoints)

    def test_create_preserves_fields(self, api, clean_auton_waypoints):
        wp = _create_waypoint(api, name='field-test', tag_id=99, type=2, lat=10.0, lon=20.0, coverage_radius=3.5)
        assert wp['name'] == 'field-test'
        assert wp['tag_id'] == 99
        assert wp['type'] == 2
        assert abs(wp['lat'] - 10.0) < 0.001
        assert abs(wp['lon'] - 20.0) < 0.001
        assert wp['coverage_radius'] == 3.5

    def test_create_does_not_affect_defaults(self, api, clean_auton_waypoints):
        _create_waypoint(api)
        waypoints = _get_waypoints(api)
        defaults = [w for w in waypoints if not w['deletable']]
        assert len(defaults) == 8


class TestAutonWaypointUpdate:
    def test_partial_update(self, api, clean_auton_waypoints):
        wp = _create_waypoint(api, name='original')
        resp = api.patch(url(api, f'/api/waypoints/auton/{wp["db_id"]}/'), json={'name': 'updated'})
        assert resp.status_code == 200
        data = resp.json()
        assert data['waypoint']['name'] == 'updated'
        assert data['waypoint']['tag_id'] == wp['tag_id']

    def test_update_default_waypoint(self, api):
        resp = api.patch(url(api, '/api/waypoints/auton/1/'), json={'lat': 42.999, 'lon': -83.999})
        assert resp.status_code == 200
        data = resp.json()
        assert abs(data['waypoint']['lat'] - 42.999) < 0.001
        assert abs(data['waypoint']['lon'] - (-83.999)) < 0.001

    def test_update_nonexistent_returns_404(self, api):
        resp = api.patch(url(api, '/api/waypoints/auton/9999/'), json={'name': 'nope'})
        assert resp.status_code == 404

    def test_update_empty_body_returns_400(self, api):
        resp = api.patch(url(api, '/api/waypoints/auton/1/'), json={})
        assert resp.status_code == 400

    def test_update_preserves_unchanged_fields(self, api, clean_auton_waypoints):
        wp = _create_waypoint(api, name='keep-fields', tag_id=77, coverage_radius=2.0)
        api.patch(url(api, f'/api/waypoints/auton/{wp["db_id"]}/'), json={'name': 'renamed'})
        waypoints = _get_waypoints(api)
        updated = next(w for w in waypoints if w['db_id'] == wp['db_id'])
        assert updated['name'] == 'renamed'
        assert updated['tag_id'] == 77
        assert updated['coverage_radius'] == 2.0


class TestAutonWaypointDelete:
    def test_delete_user_waypoint(self, api, clean_auton_waypoints):
        wp = _create_waypoint(api)
        db_id = wp['db_id']
        resp = api.delete(url(api, f'/api/waypoints/auton/{db_id}/'))
        assert resp.status_code == 200
        waypoints = _get_waypoints(api)
        assert not any(w['db_id'] == db_id for w in waypoints)

    def test_delete_default_returns_403(self, api):
        resp = api.delete(url(api, '/api/waypoints/auton/1/'))
        assert resp.status_code == 403

    def test_delete_nonexistent_returns_404(self, api):
        resp = api.delete(url(api, '/api/waypoints/auton/9999/'))
        assert resp.status_code == 404

    def test_clear_removes_only_user_waypoints(self, api, clean_auton_waypoints):
        _create_waypoint(api)
        api.delete(url(api, '/api/waypoints/auton/clear/'))
        waypoints = _get_waypoints(api)
        user_wps = [w for w in waypoints if w['deletable']]
        defaults = [w for w in waypoints if not w['deletable']]
        assert len(user_wps) == 0
        assert len(defaults) == 8

    def test_clear_all_removes_everything(self, api):
        api.delete(url(api, '/api/waypoints/auton/clear/all/'))
        resp = api.get(url(api, '/api/waypoints/auton/'))
        data = resp.json()
        assert len(data['waypoints']) == 0
        seed_auton_defaults()

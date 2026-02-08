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


def _save_waypoints(api, waypoints):
    resp = api.post(url(api, '/api/waypoints/auton/save/'), json={'waypoints': waypoints})
    assert resp.status_code == 200
    return resp.json()


def _make_user_waypoint(**overrides):
    wp = {
        'name': 'Test WP',
        'tag_id': 42,
        'type': 1,
        'lat': 42.123,
        'lon': -83.456,
        'enable_costmap': True,
        'coverage_radius': 0.0,
        'deletable': True,
    }
    wp.update(overrides)
    return wp


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


class TestAutonWaypointCRUD:
    def test_save_preserves_tag_id(self, api, clean_auton_waypoints):
        user_wp = _make_user_waypoint(tag_id=42)
        _save_waypoints(api, [user_wp])
        waypoints = _get_waypoints(api)
        user_wps = [w for w in waypoints if w['deletable']]
        assert any(w['tag_id'] == 42 for w in user_wps)

    def test_save_preserves_name(self, api, clean_auton_waypoints):
        user_wp = _make_user_waypoint(name='my-test-wp')
        _save_waypoints(api, [user_wp])
        waypoints = _get_waypoints(api)
        user_wps = [w for w in waypoints if w['deletable']]
        assert any(w['name'] == 'my-test-wp' for w in user_wps)

    def test_save_preserves_coverage_radius(self, api, clean_auton_waypoints):
        user_wp = _make_user_waypoint(coverage_radius=3.5)
        _save_waypoints(api, [user_wp])
        waypoints = _get_waypoints(api)
        user_wps = [w for w in waypoints if w['deletable']]
        assert any(w['coverage_radius'] == 3.5 for w in user_wps)

    def test_save_preserves_type(self, api, clean_auton_waypoints):
        user_wp = _make_user_waypoint(type=2)
        _save_waypoints(api, [user_wp])
        waypoints = _get_waypoints(api)
        user_wps = [w for w in waypoints if w['deletable']]
        assert any(w['type'] == 2 for w in user_wps)

    def test_save_does_not_remove_defaults(self, api, clean_auton_waypoints):
        user_wp = _make_user_waypoint()
        _save_waypoints(api, [user_wp])
        waypoints = _get_waypoints(api)
        defaults = [w for w in waypoints if not w['deletable']]
        assert len(defaults) == 8

    def test_update_default_waypoint_lat_lon(self, api):
        waypoints = _get_waypoints(api)
        default_wp = next(w for w in waypoints if w['db_id'] == 1)
        updated = {
            'name': default_wp['name'],
            'tag_id': default_wp['tag_id'],
            'type': default_wp['type'],
            'lat': 42.999,
            'lon': -83.999,
            'enable_costmap': default_wp['enable_costmap'],
            'coverage_radius': default_wp['coverage_radius'],
            'deletable': False,
            'db_id': 1,
        }
        _save_waypoints(api, [updated])
        refreshed = _get_waypoints(api)
        wp1 = next(w for w in refreshed if w['db_id'] == 1)
        assert abs(wp1['lat'] - 42.999) < 0.001
        assert abs(wp1['lon'] - (-83.999)) < 0.001


class TestAutonWaypointDelete:
    def test_delete_user_waypoint(self, api, clean_auton_waypoints):
        user_wp = _make_user_waypoint()
        _save_waypoints(api, [user_wp])
        waypoints = _get_waypoints(api)
        user_wps = [w for w in waypoints if w['deletable']]
        assert len(user_wps) >= 1
        db_id = user_wps[0]['db_id']
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
        user_wp = _make_user_waypoint()
        _save_waypoints(api, [user_wp])
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

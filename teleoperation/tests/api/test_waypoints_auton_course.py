import pytest
from api.conftest import url

pytestmark = pytest.mark.api


def _make_course_waypoint(name='CourseWP', tag_id=10, **overrides):
    wp = {
        'name': name,
        'tag_id': tag_id,
        'type': 1,
        'lat': 42.0,
        'lon': -83.0,
        'enable_costmap': True,
        'coverage_radius': 0.0,
        'deletable': True,
    }
    wp.update(overrides)
    return wp


def _get_course(api):
    resp = api.get(url(api, '/api/waypoints/auton/current/'))
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'success'
    return data['course']


def _save_course(api, waypoints):
    resp = api.post(url(api, '/api/waypoints/auton/current/save/'), json={'waypoints': waypoints})
    assert resp.status_code == 200
    return resp.json()


class TestAutonCourse:
    def test_empty_course_initially(self, api, clean_auton_waypoints):
        course = _get_course(api)
        assert course == []

    def test_save_and_get_course(self, api, clean_auton_waypoints):
        wps = [
            _make_course_waypoint(name='A', tag_id=1),
            _make_course_waypoint(name='B', tag_id=2),
            _make_course_waypoint(name='C', tag_id=3),
        ]
        _save_course(api, wps)
        course = _get_course(api)
        assert len(course) == 3

    def test_course_preserves_tag_id(self, api, clean_auton_waypoints):
        wps = [_make_course_waypoint(tag_id=10)]
        _save_course(api, wps)
        course = _get_course(api)
        assert course[0]['tag_id'] == 10

    def test_course_preserves_sequence_order(self, api, clean_auton_waypoints):
        wps = [
            _make_course_waypoint(name='A'),
            _make_course_waypoint(name='B'),
            _make_course_waypoint(name='C'),
        ]
        _save_course(api, wps)
        course = _get_course(api)
        names = [w['name'] for w in course]
        assert names == ['A', 'B', 'C']

    def test_course_preserves_coverage_radius(self, api, clean_auton_waypoints):
        wps = [_make_course_waypoint(coverage_radius=2.0)]
        _save_course(api, wps)
        course = _get_course(api)
        assert course[0]['coverage_radius'] == 2.0

    def test_course_enable_costmap_is_bool(self, api, clean_auton_waypoints):
        wps = [_make_course_waypoint(enable_costmap=1)]
        _save_course(api, wps)
        course = _get_course(api)
        assert course[0]['enable_costmap'] is True
        assert isinstance(course[0]['enable_costmap'], bool)


import pytest
from api.conftest import url

pytestmark = pytest.mark.api


def _get_recordings(api):
    resp = api.get(url(api, '/api/recordings/'))
    assert resp.status_code == 200
    data = resp.json()
    assert data['status'] == 'success'
    return data['recordings']


class TestRecordings:
    def test_empty_initially(self, api, clean_recordings):
        recordings = _get_recordings(api)
        assert recordings == []

    def test_create_recording(self, api, clean_recordings):
        resp = api.post(url(api, '/api/recordings/create/'), json={
            'name': 'Test Recording',
            'is_drone': False,
        })
        assert resp.status_code == 200
        data = resp.json()
        assert data['status'] == 'success'
        assert 'recording_id' in data
        # Stop so it doesn't interfere with other tests
        api.post(url(api, '/api/recordings/stop/'))

    def test_stop_recording(self, api, clean_recordings):
        api.post(url(api, '/api/recordings/create/'), json={
            'name': 'Stop Test',
            'is_drone': False,
        })
        resp = api.post(url(api, '/api/recordings/stop/'))
        assert resp.status_code == 200
        data = resp.json()
        assert data['status'] == 'success'

    def test_stop_without_active_returns_error(self, api, clean_recordings):
        resp = api.post(url(api, '/api/recordings/stop/'))
        assert resp.status_code in (400, 500)

    def test_add_waypoint_to_recording(self, api, clean_recordings):
        create_resp = api.post(url(api, '/api/recordings/create/'), json={
            'name': 'WP Test',
            'is_drone': False,
        })
        rec_id = create_resp.json()['recording_id']
        resp = api.post(url(api, f'/api/recordings/{rec_id}/waypoints/'), json={
            'lat': 42.0,
            'lon': -83.0,
            'sequence': 0,
        })
        assert resp.status_code == 200
        api.post(url(api, '/api/recordings/stop/'))

    def test_get_recording_waypoints(self, api, clean_recordings):
        create_resp = api.post(url(api, '/api/recordings/create/'), json={
            'name': 'Get WP Test',
            'is_drone': False,
        })
        rec_id = create_resp.json()['recording_id']
        api.post(url(api, f'/api/recordings/{rec_id}/waypoints/'), json={
            'lat': 42.0, 'lon': -83.0, 'sequence': 0,
        })
        api.post(url(api, '/api/recordings/stop/'))

        resp = api.get(url(api, f'/api/recordings/{rec_id}/waypoints/'))
        assert resp.status_code == 200
        data = resp.json()
        assert data['status'] == 'success'
        assert len(data['waypoints']) == 1

    def test_delete_recording(self, api, clean_recordings):
        create_resp = api.post(url(api, '/api/recordings/create/'), json={
            'name': 'Delete Test',
            'is_drone': False,
        })
        rec_id = create_resp.json()['recording_id']
        api.post(url(api, '/api/recordings/stop/'))

        resp = api.delete(url(api, f'/api/recordings/{rec_id}/'))
        assert resp.status_code == 200
        recordings = _get_recordings(api)
        assert not any(r['id'] == rec_id for r in recordings)

    def test_clear_all_recordings(self, api, clean_recordings):
        api.post(url(api, '/api/recordings/create/'), json={
            'name': 'Clear Test',
            'is_drone': False,
        })
        api.post(url(api, '/api/recordings/stop/'))
        api.delete(url(api, '/api/recordings/clear/'))
        recordings = _get_recordings(api)
        assert recordings == []

import pytest
from api.conftest import url

pytestmark = pytest.mark.api


class TestTeleop:
    def test_enable_teleop(self, api):
        resp = api.post(url(api, '/api/enable_teleop/'), json={'enabled': True})
        assert resp.status_code == 200
        data = resp.json()
        assert data['status'] == 'success'

    def test_disable_teleop(self, api):
        resp = api.post(url(api, '/api/enable_teleop/'), json={'enabled': False})
        assert resp.status_code == 200
        data = resp.json()
        assert data['status'] == 'success'

    def test_response_echoes_enabled(self, api):
        resp = api.post(url(api, '/api/enable_teleop/'), json={'enabled': True})
        data = resp.json()
        assert data['enabled'] is True

        resp = api.post(url(api, '/api/enable_teleop/'), json={'enabled': False})
        data = resp.json()
        assert data['enabled'] is False

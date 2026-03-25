import pytest
from api.conftest import url

pytestmark = pytest.mark.api


class TestArmMode:
    def test_set_disabled(self, api):
        resp = api.post(url(api, '/api/arm/ra_mode/'), json={'mode': 'disabled'})
        assert resp.status_code == 200

    def test_set_throttle(self, api):
        resp = api.post(url(api, '/api/arm/ra_mode/'), json={'mode': 'throttle'})
        assert resp.status_code == 200

    def test_set_ik_pos(self, api):
        resp = api.post(url(api, '/api/arm/ra_mode/'), json={'mode': 'ik-pos'})
        assert resp.status_code == 200

    def test_set_ik_vel(self, api):
        resp = api.post(url(api, '/api/arm/ra_mode/'), json={'mode': 'ik-vel'})
        assert resp.status_code == 200

    def test_response_echoes_mode(self, api):
        resp = api.post(url(api, '/api/arm/ra_mode/'), json={'mode': 'throttle'})
        data = resp.json()
        assert data['mode'] == 'throttle'

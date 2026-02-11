import os
import sqlite3

import pytest
import requests

TELEOP_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
WAYPOINTS_DB = os.path.join(TELEOP_DIR, 'waypoints.db')

AUTON_DEFAULTS = [
    ('No Search 1', 0, 0, 0.0, 0.0, 1, 0.0, 0),
    ('No Search 2', 1, 0, 0.0, 0.0, 1, 0.0, 0),
    ('Post 1', 2, 1, 0.0, 0.0, 1, 0.0, 0),
    ('Post 2', 3, 1, 0.0, 0.0, 1, 0.0, 0),
    ('Post 3', 4, 1, 0.0, 0.0, 1, 0.0, 0),
    ('Mallet', 5, 2, 0.0, 0.0, 1, 0.0, 0),
    ('Water Bottle', 6, 3, 0.0, 0.0, 1, 0.0, 0),
    ('Rock Pick', 7, 4, 0.0, 0.0, 1, 0.0, 0),
]


def seed_auton_defaults():
    conn = sqlite3.connect(WAYPOINTS_DB)
    conn.row_factory = sqlite3.Row
    count = conn.execute('SELECT count(*) FROM auton_waypoints WHERE deletable = 0').fetchone()[0]
    if count < 8:
        conn.execute('DELETE FROM auton_waypoints')
        conn.execute('DELETE FROM current_auton_course')
        conn.execute('DELETE FROM sqlite_sequence WHERE name = "auton_waypoints"')
        for i, d in enumerate(AUTON_DEFAULTS, start=1):
            conn.execute(
                'INSERT INTO auton_waypoints (id, name, tag_id, type, latitude, longitude, enable_costmap, coverage_radius, deletable) '
                'VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)',
                (i, *d),
            )
        conn.execute('DELETE FROM sqlite_sequence WHERE name = "auton_waypoints"')
        conn.execute('INSERT INTO sqlite_sequence (name, seq) VALUES ("auton_waypoints", 8)')
        conn.commit()
    conn.close()


@pytest.fixture(scope='session')
def backend_url():
    return 'http://localhost:8000'


@pytest.fixture(scope='session')
def api(backend_url):
    session = requests.Session()
    session.base_url = backend_url
    return session


@pytest.fixture(scope='session', autouse=True)
def ensure_auton_defaults():
    seed_auton_defaults()


def _url(api, path):
    return f'{api.base_url}{path}'


@pytest.fixture
def clean_auton_waypoints(api):
    api.delete(_url(api, '/api/waypoints/auton/clear/'))
    yield
    api.delete(_url(api, '/api/waypoints/auton/clear/'))


@pytest.fixture
def clean_basic_waypoints(api):
    api.delete(_url(api, '/api/waypoints/basic/clear/'))
    yield
    api.delete(_url(api, '/api/waypoints/basic/clear/'))


@pytest.fixture
def clean_recordings(api):
    api.delete(_url(api, '/api/recordings/clear/'))
    yield
    api.delete(_url(api, '/api/recordings/clear/'))


def url(api, path):
    return f'{api.base_url}{path}'

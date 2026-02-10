"""
Shared pytest fixtures for teleoperation tests.
"""

import pytest


def pytest_configure(config):
    """Register custom markers."""
    config.addinivalue_line(
        'markers', 'integration: marks tests as integration tests (require running services)'
    )
    config.addinivalue_line(
        'markers', 'ros: marks tests that require ROS 2 environment'
    )


@pytest.fixture(scope='session')
def backend_url():
    """Base URL for the backend server."""
    return 'http://localhost:8000'


@pytest.fixture(scope='session')
def frontend_url():
    """Base URL for the frontend dev server."""
    return 'http://localhost:8080'


@pytest.fixture(scope='session')
def ws_base_url():
    """Base WebSocket URL for the backend server."""
    return 'ws://localhost:8000'

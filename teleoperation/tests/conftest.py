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
    config.addinivalue_line(
        'markers', 'api: marks tests as API contract tests (require backend only)'
    )



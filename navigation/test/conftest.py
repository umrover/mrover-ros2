import os

def pytest_configure(config):
    if os.environ.get("PYTEST_COV") != "1":
        cov = config.pluginmanager.get_plugin('_cov')
        cov.options.no_cov = True
        if cov.cov_controller:
            cov.cov_controller.pause()

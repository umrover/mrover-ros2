import logging

class ROS2Formatter(logging.Formatter):
    def format(self, record):
        return f"[{record.levelname.upper()}] {record.getMessage()}"

LOGGING_CONFIG = {
    "version": 1,
    "disable_existing_loggers": False,
    "formatters": {
        "ros2": {
            "()": "backend.logging_config.ROS2Formatter",
        },
    },
    "handlers": {
        "default": {
            "formatter": "ros2",
            "class": "logging.StreamHandler",
            "stream": "ext://sys.stderr",
        },
    },
    "loggers": {
        "uvicorn": {"handlers": ["default"], "level": "INFO"},
        "uvicorn.error": {"handlers": ["default"], "level": "INFO", "propagate": False},
        "uvicorn.access": {"handlers": ["default"], "level": "INFO", "propagate": False},
    },
}

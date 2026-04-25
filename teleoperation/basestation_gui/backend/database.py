import logging
import sqlite3
from pathlib import Path

logger = logging.getLogger(__name__)

BASE_DIR = Path(__file__).resolve().parent.parent.parent
BASE_DIR.mkdir(parents=True, exist_ok=True)

WAYPOINTS_DB = BASE_DIR / 'waypoints.db'
RECORDINGS_DB = BASE_DIR / 'recordings.db'

def init_waypoints_db():
    try:
        conn = sqlite3.connect(WAYPOINTS_DB)
        cursor = conn.cursor()

        cursor.execute("PRAGMA foreign_keys = ON;")

        cursor.execute('''
            CREATE TABLE IF NOT EXISTS auton_waypoints (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL,
                tag_id INTEGER DEFAULT NULL,
                type INTEGER DEFAULT 0,
                latitude REAL DEFAULT 0.0,
                longitude REAL DEFAULT 0.0,
                enable_costmap BOOLEAN DEFAULT 1,
                coverage_radius REAL DEFAULT 0.0,
                deletable BOOLEAN DEFAULT 1
            )
        ''')

        cursor.execute('''
            CREATE TABLE IF NOT EXISTS current_auton_course (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL,
                tag_id INTEGER DEFAULT NULL,
                type INTEGER DEFAULT 0,
                latitude REAL DEFAULT 0.0,
                longitude REAL DEFAULT 0.0,
                enable_costmap BOOLEAN DEFAULT 1,
                coverage_radius REAL DEFAULT 0.0,
                sequence_order INTEGER NOT NULL
            )
        ''')

        cursor.execute('''
            CREATE TABLE IF NOT EXISTS basic_waypoints (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL,
                latitude REAL NOT NULL,
                longitude REAL NOT NULL,
                drone BOOLEAN DEFAULT 0
            )
        ''')

        cursor.execute("SELECT count(*) FROM auton_waypoints")
        if cursor.fetchone()[0] == 0:
            defaults = [
                ("No Search 1", None, 0, 0.0, 0.0, 1, 0.0, 0),
                ("No Search 2", None, 0, 0.0, 0.0, 1, 0.0, 0),
                ("Post 1", 1, 1, 0.0, 0.0, 1, 0.0, 0),
                ("Post 2", 2, 1, 0.0, 0.0, 1, 0.0, 0),
                ("Post 3", 3, 1, 0.0, 0.0, 1, 0.0, 0),
                ("Mallet", None, 2, 0.0, 0.0, 1, 0.0, 0),
                ("Water Bottle", None, 3, 0.0, 0.0, 1, 0.0, 0),
                ("Rock Pick", None, 4, 0.0, 0.0, 1, 0.0, 0),
            ]
            cursor.executemany('''
                INSERT INTO auton_waypoints (name, tag_id, type, latitude, longitude, enable_costmap, coverage_radius, deletable)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            ''', defaults)

            cursor.execute('DELETE FROM sqlite_sequence WHERE name = "auton_waypoints"')
            cursor.execute('INSERT INTO sqlite_sequence (name, seq) VALUES ("auton_waypoints", 8)')

        conn.commit()
        conn.close()
        logger.info(f"Waypoints database initialized: {WAYPOINTS_DB}")
    except Exception as e:
        logger.error(f"Error initializing waypoints database: {e}")
        raise

def init_recordings_db():
    try:
        conn = sqlite3.connect(RECORDINGS_DB)
        cursor = conn.cursor()

        cursor.execute("PRAGMA foreign_keys = ON;")

        cursor.execute('''
            CREATE TABLE IF NOT EXISTS recordings (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL,
                is_drone BOOLEAN DEFAULT 0,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')

        cursor.execute('''
            CREATE TABLE IF NOT EXISTS recorded_waypoints (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                recording_id INTEGER NOT NULL,
                latitude REAL NOT NULL,
                longitude REAL NOT NULL,
                sequence INTEGER NOT NULL,
                timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                FOREIGN KEY(recording_id) REFERENCES recordings(id) ON DELETE CASCADE
            )
        ''')

        conn.commit()
        conn.close()
        logger.info(f"Recordings database initialized: {RECORDINGS_DB}")
    except Exception as e:
        logger.error(f"Error initializing recordings database: {e}")
        raise

def get_waypoints_db():
    conn = sqlite3.connect(WAYPOINTS_DB)
    conn.row_factory = sqlite3.Row
    return conn

def get_recordings_db():
    conn = sqlite3.connect(RECORDINGS_DB)
    conn.row_factory = sqlite3.Row
    return conn

def get_db_connection():
    return get_waypoints_db()

_initialized = False

def ensure_initialized():
    global _initialized
    if not _initialized:
        init_waypoints_db()
        init_recordings_db()
        _initialized = True

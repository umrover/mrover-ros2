import sqlite3
import os

BASE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../..')

# Ensure base directory exists
os.makedirs(BASE_DIR, exist_ok=True)

WAYPOINTS_DB = os.path.join(BASE_DIR, 'waypoints.db')
RECORDINGS_DB = os.path.join(BASE_DIR, 'recordings.db')

def init_waypoints_db():
    try:
        conn = sqlite3.connect(WAYPOINTS_DB)
        cursor = conn.cursor()

        # Enable foreign keys
        cursor.execute("PRAGMA foreign_keys = ON;")

        # Auton Waypoints Table
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS auton_waypoints (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL,
                tag_id INTEGER DEFAULT -1,
                type INTEGER DEFAULT 0,
                latitude REAL DEFAULT 0.0,
                longitude REAL DEFAULT 0.0,
                enable_costmap BOOLEAN DEFAULT 1,
                coverage_radius REAL DEFAULT 0.0,
                deletable BOOLEAN DEFAULT 1
            )
        ''')

        # Current Auton Course Table
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS current_auton_course (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL,
                tag_id INTEGER DEFAULT -1,
                type INTEGER DEFAULT 0,
                latitude REAL DEFAULT 0.0,
                longitude REAL DEFAULT 0.0,
                enable_costmap BOOLEAN DEFAULT 1,
                coverage_radius REAL DEFAULT 0.0,
                sequence_order INTEGER NOT NULL
            )
        ''')

        # Basic Waypoints Table
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS basic_waypoints (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL,
                latitude REAL NOT NULL,
                longitude REAL NOT NULL,
                drone BOOLEAN DEFAULT 0
            )
        ''')

        # Seed default data if table is empty
        cursor.execute("SELECT count(*) FROM auton_waypoints")
        if cursor.fetchone()[0] == 0:
            defaults = [
                ("No Search 1", 0, 0, 0.0, 0.0, 1, 0.0, 0), # Not deletable
                ("No Search 2", 1, 0, 0.0, 0.0, 1, 0.0, 0), # Not deletable
                ("Post 1", 2, 1, 0.0, 0.0, 1, 0.0, 0),      # Not deletable
                ("Post 2", 3, 1, 0.0, 0.0, 1, 0.0, 0),      # Not deletable
                ("Post 3", 4, 1, 0.0, 0.0, 1, 0.0, 0),      # Not deletable
                ("Mallet", 5, 2, 0.0, 0.0, 1, 0.0, 0),      # Not deletable
                ("Water Bottle", 6, 3, 0.0, 0.0, 1, 0.0, 0),# Not deletable
                ("Rock Pick", 7, 4, 0.0, 0.0, 1, 0.0, 0),   # Not deletable
            ]
            cursor.executemany('''
                INSERT INTO auton_waypoints (name, tag_id, type, latitude, longitude, enable_costmap, coverage_radius, deletable)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?)
            ''', defaults)

            # Set auto-increment to 8
            cursor.execute('DELETE FROM sqlite_sequence WHERE name = "auton_waypoints"')
            cursor.execute('INSERT INTO sqlite_sequence (name, seq) VALUES ("auton_waypoints", 8)')

        conn.commit()
        conn.close()
        print(f"Waypoints database initialized: {WAYPOINTS_DB}")
    except Exception as e:
        print(f"Error initializing waypoints database: {e}")
        raise

def init_recordings_db():
    try:
        conn = sqlite3.connect(RECORDINGS_DB)
        cursor = conn.cursor()

        # Enable foreign keys
        cursor.execute("PRAGMA foreign_keys = ON;")

        # Recordings Table
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS recordings (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL,
                is_drone BOOLEAN DEFAULT 0,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        ''')

        # Recorded Waypoints Table
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
        print(f"Recordings database initialized: {RECORDINGS_DB}")
    except Exception as e:
        print(f"Error initializing recordings database: {e}")
        raise

def get_waypoints_db():
    conn = sqlite3.connect(WAYPOINTS_DB)
    conn.row_factory = sqlite3.Row
    return conn

def get_recordings_db():
    conn = sqlite3.connect(RECORDINGS_DB)
    conn.row_factory = sqlite3.Row
    return conn

# For backwards compatibility
def get_db_connection():
    return get_waypoints_db()

# Initialize on module load
init_waypoints_db()
init_recordings_db()

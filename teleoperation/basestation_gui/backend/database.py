import sqlite3
import os

DB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../mrover.db')

def init_db():
    conn = sqlite3.connect(DB_FILE)
    cursor = conn.cursor()
    
    # Enable foreign keys
    cursor.execute("PRAGMA foreign_keys = ON;")

    # Auton Waypoints Table (The "Store")
    # Added "deletable" field (default 1/True). 
    # Pre-seeded items like Mallet/WaterBottle will have deletable=0.
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS auton_waypoints (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT NOT NULL,
            tag_id INTEGER DEFAULT -1,
            type INTEGER DEFAULT 0,
            latitude REAL DEFAULT 0.0,
            longitude REAL DEFAULT 0.0,
            enable_costmap BOOLEAN DEFAULT 1,
            deletable BOOLEAN DEFAULT 1
        )
    ''')

    # Current Auton Course Table (The "Active Route")
    # This references waypoints from the store, but since the order matters and 
    # the same waypoint might appear multiple times (though less likely for physical objects),
    # we'll just copy the data or reference the ID. 
    # Given the previous implementation copied data, let's keep a separate table 
    # but maybe link it conceptually.
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS current_auton_course (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT NOT NULL,
            tag_id INTEGER DEFAULT -1,
            type INTEGER DEFAULT 0,
            latitude REAL DEFAULT 0.0,
            longitude REAL DEFAULT 0.0,
            enable_costmap BOOLEAN DEFAULT 1,
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

    # Seed default data if table is empty
    cursor.execute("SELECT count(*) FROM auton_waypoints")
    if cursor.fetchone()[0] == 0:
        defaults = [
            ("No Search 1", -1, 0, 0.0, 0.0, 1, 0), # Not deletable
            ("No Search 2", -1, 0, 0.0, 0.0, 1, 0), # Not deletable
            ("Post 1", 1, 1, 0.0, 0.0, 1, 0),       # Not deletable
            ("Post 2", 2, 1, 0.0, 0.0, 1, 0),       # Not deletable
            ("Post 3", 3, 1, 0.0, 0.0, 1, 0),       # Not deletable
            ("Mallet", -1, 2, 0.0, 0.0, 1, 0),      # Not deletable
            ("Water Bottle", -1, 3, 0.0, 0.0, 1, 0),# Not deletable
            ("Rock Pick", -1, 4, 0.0, 0.0, 1, 0),   # Not deletable
        ]
        cursor.executemany('''
            INSERT INTO auton_waypoints (name, tag_id, type, latitude, longitude, enable_costmap, deletable)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        ''', defaults)

    conn.commit()
    conn.close()

def get_db_connection():
    conn = sqlite3.connect(DB_FILE)
    conn.row_factory = sqlite3.Row
    return conn

# Initialize on module load
init_db()

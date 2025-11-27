import os
import json
from flask import Blueprint, jsonify, request

waypoints_bp = Blueprint('waypoints', __name__)

# Waypoint Storage
WAYPOINTS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../waypoints.json')

def load_waypoints():
    if os.path.exists(WAYPOINTS_FILE):
        try:
            with open(WAYPOINTS_FILE, 'r') as f:
                return json.load(f)
        except json.JSONDecodeError:
            pass
    return {'basic': [], 'auton': [], 'current_course': []}

def save_waypoints(data):
    with open(WAYPOINTS_FILE, 'w') as f:
        json.dump(data, f, indent=2)

# Initialize Waypoints
waypoints_data = load_waypoints()

@waypoints_bp.route('/basic/', methods=['GET'])
def get_basic_waypoints():
    return jsonify({'status': 'success', 'waypoints': waypoints_data.get('basic', [])})

@waypoints_bp.route('/basic/save/', methods=['POST'])
def save_basic_waypoints():
    data = request.json
    waypoints_data['basic'] = data.get('waypoints', [])
    save_waypoints(waypoints_data)
    return jsonify({'status': 'success', 'waypoints': waypoints_data['basic']})

@waypoints_bp.route('/basic/clear/', methods=['DELETE'])
def clear_basic_waypoints():
    waypoints_data['basic'] = []
    save_waypoints(waypoints_data)
    return jsonify({'status': 'success', 'waypoints': []})

@waypoints_bp.route('/auton/', methods=['GET'])
def get_auton_waypoints():
    return jsonify({'status': 'success', 'waypoints': waypoints_data.get('auton', [])})

@waypoints_bp.route('/auton/save/', methods=['POST'])
def save_auton_waypoints():
    data = request.json
    waypoints_data['auton'] = data.get('waypoints', [])
    save_waypoints(waypoints_data)
    return jsonify({'status': 'success', 'waypoints': waypoints_data['auton']})

@waypoints_bp.route('/auton/current/', methods=['GET'])
def get_current_auton_course():
    return jsonify({'status': 'success', 'course': waypoints_data.get('current_course', [])})

@waypoints_bp.route('/auton/current/save/', methods=['POST'])
def save_current_auton_course():
    data = request.json
    waypoints_data['current_course'] = data.get('course', [])
    save_waypoints(waypoints_data)
    return jsonify({'status': 'success', 'course': waypoints_data['current_course']})

@waypoints_bp.route('/auton/<int:waypoint_id>/', methods=['DELETE'])
def delete_auton_waypoint(waypoint_id):
    # Filter out the waypoint with the given ID
    initial_len = len(waypoints_data['auton'])
    waypoints_data['auton'] = [wp for wp in waypoints_data['auton'] if wp.get('id') != waypoint_id]
    
    if len(waypoints_data['auton']) < initial_len:
        save_waypoints(waypoints_data)
        return jsonify({'status': 'success', 'message': f'Waypoint {waypoint_id} deleted'})
    else:
        return jsonify({'status': 'error', 'message': 'Waypoint not found'}), 404

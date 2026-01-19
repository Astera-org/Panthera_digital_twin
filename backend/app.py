#!/usr/bin/env python3
"""
Digital Twin Backend - Flask + WebSocket Server
Connects to Panthera robot and streams real-time data to web interface.

Usage:
    python app.py                                   # Uses default config
    python app.py --config path/to/robot.yaml       # Custom config
    python app.py --demo                            # Demo mode without robot
"""
import sys
import os
import time
import threading
import logging
import argparse
import yaml
import numpy as np
import base64
from flask import Flask, jsonify, request, send_from_directory
from flask_socketio import SocketIO, emit
from flask_cors import CORS

# Import camera streamer
from camera import CameraStreamer

# Disable Flask's request logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

# Add Panthera SDK to path (Panthera_SDK_V6 contains the latest version)
SDK_PATH = os.path.join(os.path.dirname(__file__), '..', '..', 'Panthera_SDK_V6', 'panthera_python')
sys.path.append(SDK_PATH)
sys.path.append(os.path.join(SDK_PATH, 'scripts'))

# Local config path (self-contained in digital_twin folder)
LOCAL_CONFIG_PATH = os.path.join(os.path.dirname(__file__), '..', 'robot_param', 'xlb.yaml')

app = Flask(__name__, static_folder='../frontend/dist', static_url_path='')
CORS(app, origins="*")
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# ============== GLOBAL SETTINGS ==============
CONTROL_FREQ = 200  # Hz - control loop frequency
BROADCAST_FREQ = 30  # Hz - WebSocket broadcast frequency
END_EFFECTOR_OFFSET = 0.140  # meters - offset from Link_6 origin to actual tool tip
# =============================================

# Robot instance
robot = None
robot_config = None
demo_mode = False

# Control state
target_positions = [0.0] * 6
target_velocity = 0.5
max_torque = [10.0, 10.0, 10.0, 10.0, 10.0, 2.0]

# Current state
current_positions = [0.0] * 6
current_velocities = [0.0] * 6
current_torques = [0.0] * 6

# ============== CONTROL MODE SETTINGS ==============
# Modes: 'position', 'gravity_comp', 'impedance'
control_mode = 'position'


# Gravity compensation parameters
gravity_gain = np.array([1.0, 1.0, 1.0, 1.2, 1.0, 1.0])
joint_offset = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
tau_limit = np.array([10.0, 10.0, 10.0, 10.0, 10.0, 3.7])

# Impedance control parameters (PD + gravity)
impedance_K = np.array([5.0, 5.0, 5.0, 5.0, 5.0, 1.0])  # Stiffness
impedance_B = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.1])    # Damping
impedance_target = np.array([0.0, 0.7, 0.7, -0.1, 0.0, 0.0])  # Target position

# Joint configuration
JOINT_CONFIG = []
URDF_PATH = None

# Thread control
target_lock = threading.Lock()
loop_running = False
connected_clients = set()

# Forward kinematics data
current_fk = {
    'position': [0.0, 0.0, 0.0],
    'euler': [0.0, 0.0, 0.0],  # Roll, Pitch, Yaw in degrees
    'rotation': [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
}

# Waypoints for trajectory execution (max 6)
MAX_WAYPOINTS = 6
waypoints = []
trajectory_running = False
trajectory_progress = 0.0

# ============== CAMERA SETTINGS ==============
camera_streamer = None
camera_streaming = False
CAMERA_BROADCAST_FREQ = 60  # Hz - image stream frequency

# ============== CALIBRATION DATA ==============
# Stores pairs of (camera_frame_position, robot_frame_position) for hand-eye calibration
calibration_pairs = []
last_camera_position = None  # Stores the most recent camera depth query result

# ============== CAM2BOT TRANSFORMATION ==============
# Camera to robot base transformation matrix (loaded from cam2bot.yaml)
cam2bot_R = None  # 3x3 rotation matrix
cam2bot_t = None  # 3x1 translation vector


def load_cam2bot_transform():
    """Load camera-to-robot transformation from cam2bot.yaml."""
    global cam2bot_R, cam2bot_t

    cam2bot_path = os.path.join(os.path.dirname(__file__), 'camera', 'cam2bot.yaml')

    if not os.path.exists(cam2bot_path):
        print(f"[Cam2Bot] Transform file not found: {cam2bot_path}")
        return False

    try:
        with open(cam2bot_path, 'r') as f:
            data = yaml.safe_load(f)

        transform = data.get('camera_to_robot_transform', {})
        cam2bot_R = np.array(transform.get('rotation_matrix', []))
        cam2bot_t = np.array(transform.get('translation', []))

        if cam2bot_R.shape != (3, 3) or cam2bot_t.shape != (3,):
            print("[Cam2Bot] Invalid transform dimensions")
            cam2bot_R = None
            cam2bot_t = None
            return False

        print(f"[Cam2Bot] Transform loaded successfully")
        print(f"  Translation: [{cam2bot_t[0]:.4f}, {cam2bot_t[1]:.4f}, {cam2bot_t[2]:.4f}]")
        return True

    except Exception as e:
        print(f"[Cam2Bot] Failed to load transform: {e}")
        return False


def apply_cam2bot_transform(point_camera):
    """Transform a point from camera frame to robot base frame.

    Args:
        point_camera: [x, y, z] in camera frame (meters)

    Returns:
        [x, y, z] in robot base frame (meters), or None if transform not loaded
    """
    global cam2bot_R, cam2bot_t

    if cam2bot_R is None or cam2bot_t is None:
        return None

    p_cam = np.array(point_camera)
    p_robot = cam2bot_R @ p_cam + cam2bot_t
    return p_robot.tolist()


def rotation_matrix_to_euler(R):
    """Convert rotation matrix to euler angles (ZYX intrinsic order, degrees)

    Uses scipy.spatial.transform.Rotation for robust conversion.
    ZYX intrinsic = Yaw-Pitch-Roll convention.
    Returns: [roll, pitch, yaw] in degrees

    Note: For this robot's coordinate frame:
    - Roll = Y rotation (lateral tilt)
    - Pitch = X rotation (forward/back tilt)
    - Yaw = Z rotation (heading)
    """
    from scipy.spatial.transform import Rotation

    R = np.array(R)
    rot = Rotation.from_matrix(R)
    # ZYX intrinsic returns [Z, Y, X] = [yaw, pitch, roll] order
    zyx_angles = rot.as_euler('ZYX', degrees=True)
    # Swap to match robot's coordinate frame convention
    roll = zyx_angles[1]   # Y rotation
    pitch = zyx_angles[2]  # X rotation
    yaw = zyx_angles[0]    # Z rotation
    return [roll, pitch, yaw]


def apply_end_effector_offset(position, rotation, offset):
    """Apply end effector offset along Link_6's local Z-axis.

    Args:
        position: FK position (Link_6 origin in world frame), numpy array [x, y, z]
        rotation: FK rotation matrix (3x3), Link_6 orientation in world frame
        offset: End effector offset distance in meters

    Returns:
        Position of actual tool tip in world frame, numpy array [x, y, z]
    """
    position = np.array(position)
    rotation = np.array(rotation)

    # Z-axis of Link_6 in world frame is the third column of rotation matrix
    z_axis = rotation[:, 2]

    # Apply offset along Link_6's local Z-axis
    return position + offset * z_axis

# Timing stats
timing_stats = {
    "loop_count": 0,
    "avg_cmd_time": 0.0,
    "overruns": 0
}


def precise_sleep(duration):
    """High precision sleep function"""
    if duration <= 0:
        return

    end_time = time.perf_counter() + duration

    # Use sleep for most of the time (leave 1ms margin)
    if duration > 0.001:
        time.sleep(duration - 0.001)

    # Busy wait for final precision
    while time.perf_counter() < end_time:
        pass


def execute_trajectory_thread(waypoint_list, durations, control_rate=100):
    """Execute trajectory in a separate thread"""
    global trajectory_running, trajectory_progress, target_positions, control_mode

    if len(waypoint_list) < 2:
        print("Need at least 2 waypoints for trajectory")
        return False

    if robot is None or demo_mode:
        print("Cannot execute trajectory: robot not available or in demo mode")
        # In demo mode, simulate trajectory execution
        if demo_mode:
            trajectory_running = True
            total_duration = sum(durations)
            elapsed = 0

            for seg_idx, duration in enumerate(durations):
                start_pos = waypoint_list[seg_idx]
                end_pos = waypoint_list[seg_idx + 1]
                steps = int(duration * 30)  # Lower rate for demo

                for step in range(steps):
                    if not trajectory_running:
                        return False

                    t = step / steps
                    # Smooth interpolation
                    s = 3 * t**2 - 2 * t**3
                    pos = [start_pos[i] + s * (end_pos[i] - start_pos[i]) for i in range(len(start_pos))]

                    with target_lock:
                        target_positions[:] = pos

                    elapsed += duration / steps
                    trajectory_progress = elapsed / total_duration
                    socketio.emit('trajectory_progress', {'progress': trajectory_progress})
                    time.sleep(1.0 / 30)

            trajectory_progress = 1.0
            socketio.emit('trajectory_progress', {'progress': 1.0})
            socketio.emit('trajectory_complete', {'success': True})
            trajectory_running = False
            return True
        return False

    # Switch to trajectory mode - control_loop will skip, letting us have exclusive control
    prev_mode = control_mode
    control_mode = 'trajectory'

    trajectory_running = True
    trajectory_progress = 0.0

    dt = 1.0 / control_rate
    total_duration = sum(durations)
    elapsed_total = 0

    try:
        for segment in range(len(durations)):
            start_pos = waypoint_list[segment]
            end_pos = waypoint_list[segment + 1]
            duration = durations[segment]

            steps = int(duration * control_rate)
            segment_start = time.perf_counter()

            for step in range(steps):
                if not trajectory_running:
                    # Trajectory was cancelled
                    socketio.emit('trajectory_complete', {'success': False, 'cancelled': True})
                    return False

                target_time = segment_start + (step + 1) * dt
                current_time = step * dt

                # Generate interpolated trajectory using septic polynomial
                pos, vel, _ = robot.septic_interpolation(start_pos, end_pos, duration, current_time)

                # Send control command
                robot.pos_vel_MAXtqe(pos, vel, max_torque)

                # Update progress
                elapsed_total = sum(durations[:segment]) + current_time
                trajectory_progress = elapsed_total / total_duration

                # Broadcast progress to clients
                socketio.emit('trajectory_progress', {'progress': trajectory_progress})

                # High precision wait
                wait_time = target_time - time.perf_counter()
                if wait_time > 0:
                    precise_sleep(wait_time)

        # Move to final position
        final_pos = waypoint_list[-1]
        robot.pos_vel_MAXtqe(final_pos, [0.0] * robot.motor_count, max_torque)

        trajectory_progress = 1.0
        socketio.emit('trajectory_progress', {'progress': 1.0})
        socketio.emit('trajectory_complete', {'success': True})

    except Exception as e:
        print(f"Trajectory execution error: {e}")
        socketio.emit('trajectory_complete', {'success': False, 'error': str(e)})

    finally:
        trajectory_running = False
        control_mode = prev_mode

    return True


def load_config(config_path):
    """Load robot configuration from YAML file"""
    global robot_config, JOINT_CONFIG, URDF_PATH

    with open(config_path, 'r', encoding='utf-8') as f:
        robot_config = yaml.safe_load(f)

    config_dir = os.path.dirname(os.path.abspath(config_path))

    # Load URDF path
    urdf_relative = robot_config['urdf']['file_path']
    URDF_PATH = os.path.normpath(os.path.join(config_dir, urdf_relative))

    # Load joint configuration
    joint_names = robot_config['kinematics']['joint_names']
    lower_limits = robot_config['robot']['joint_limits']['lower']
    upper_limits = robot_config['robot']['joint_limits']['upper']

    JOINT_CONFIG = []
    for i, name in enumerate(joint_names):
        JOINT_CONFIG.append({
            "name": name,
            "index": i,
            "min": lower_limits[i],
            "max": upper_limits[i]
        })

    print(f"Config loaded: {robot_config['robot']['name']}")
    print(f"URDF: {URDF_PATH}")
    print(f"Joints: {len(JOINT_CONFIG)}")

    return robot_config


def init_robot(config_path):
    """Initialize robot connection"""
    global robot, demo_mode

    try:
        from scripts.Panthera_lib.Panthera import Panthera

        if demo_mode:
            # In demo mode, still create Panthera instance for FK kinematics
            print("Running in DEMO mode (no motor control)")
            try:
                robot = Panthera(config_path)
                print(f"Kinematics initialized for {robot.motor_count} joints (FK available)")
            except Exception as e:
                print(f"Could not initialize kinematics: {e}")
                print("FK will not be available in demo mode")
                robot = None
            return True

        robot = Panthera(config_path)
        print(f"Robot initialized with {robot.motor_count} motors")

        # Set initial target to current position
        pos = robot.get_current_pos()
        global target_positions, current_positions
        target_positions = pos.tolist()
        current_positions = pos.tolist()

        return True
    except Exception as e:
        print(f"Failed to initialize robot: {e}")
        print("Falling back to DEMO mode")
        demo_mode = True
        return True


def control_loop():
    """Main control loop - sends commands to robot based on control mode"""
    global current_positions, current_velocities, current_torques, timing_stats
    global control_mode, impedance_target

    dt = 1.0 / CONTROL_FREQ

    # Zero arrays for torque-only control
    zero_pos = [0.0] * 6
    zero_vel = [0.0] * 6
    zero_kp = [0.0] * 6
    zero_kd = [0.0] * 6

    while loop_running:
        loop_start = time.time()

        try:
            if not demo_mode and robot is not None:
                with target_lock:
                    mode = control_mode
                    targets = target_positions.copy()
                    vel_target = target_velocity
                    imp_target = impedance_target.copy()

                t1 = time.time()

                if mode == 'position':
                    # Position control mode - direct joint control
                    vel = [vel_target] * len(targets)
                    robot.pos_vel_MAXtqe(targets, vel, max_torque, iswait=False)

                elif mode == 'gravity_comp':
                    # Gravity compensation mode - robot floats freely
                    robot.send_get_motor_state_cmd()
                    q = robot.get_current_pos() + joint_offset
                    tor = robot.get_Gravity(q) * gravity_gain
                    tor = np.clip(tor, -tau_limit, tau_limit)
                    robot.pos_vel_tqe_kp_kd(zero_pos, zero_vel, tor.tolist(), zero_kp, zero_kd)

                elif mode == 'impedance':
                    # Impedance control mode - PD + gravity compensation
                    robot.send_get_motor_state_cmd()
                    q_current = robot.get_current_pos()
                    vel_current = robot.get_current_vel()

                    # PD torque
                    tor_pd = impedance_K * (imp_target - q_current) + impedance_B * (0.0 - vel_current)

                    # Gravity compensation
                    G = robot.get_Gravity(q_current + joint_offset)

                    # Total torque
                    tor = tor_pd + G * gravity_gain
                    tor = np.clip(tor, -tau_limit, tau_limit)
                    robot.pos_vel_tqe_kp_kd(zero_pos, zero_vel, tor.tolist(), zero_kp, zero_kd)

                elif mode == 'trajectory':
                    # Trajectory mode - control handled by execute_trajectory_thread
                    # Just skip, don't send any commands
                    pass

                cmd_time = (time.time() - t1) * 1000

                timing_stats["loop_count"] += 1
                timing_stats["avg_cmd_time"] = (
                    timing_stats["avg_cmd_time"] * 0.95 + cmd_time * 0.05
                )
        except Exception as e:
            print(f"Control loop error: {e}")

        elapsed = time.time() - loop_start
        if elapsed > dt:
            timing_stats["overruns"] += 1

        sleep_time = dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


def state_broadcast_loop():
    """Broadcast robot state to connected clients"""
    global current_positions, current_velocities, current_torques, current_fk

    dt = 1.0 / BROADCAST_FREQ

    while loop_running:
        loop_start = time.time()

        try:
            if not demo_mode and robot is not None:
                # Read fresh state from robot
                robot.send_get_motor_state_cmd()
                robot.motor_send_cmd()

                pos = robot.get_current_pos()
                vel = robot.get_current_vel()
                tqe = robot.get_current_torque()

                current_positions = pos.tolist() if hasattr(pos, 'tolist') else list(pos)
                current_velocities = vel.tolist() if hasattr(vel, 'tolist') else list(vel)
                current_torques = tqe.tolist() if hasattr(tqe, 'tolist') else list(tqe)

                # Calculate forward kinematics
                # Note: Pinocchio FK should applies tool_offset=[0,0,0.14] here
                try:
                    fk = robot.forward_kinematics(pos, tool_offset=np.array([0.0, 0.0, END_EFFECTOR_OFFSET]))
                    if fk:
                        rotation = fk['rotation']
                        current_fk['position'] = fk['position'] if isinstance(fk['position'], list) else fk['position'].tolist()
                        current_fk['rotation'] = rotation.tolist() if hasattr(rotation, 'tolist') else [list(row) for row in rotation]
                        current_fk['euler'] = rotation_matrix_to_euler(rotation)

                        # Debug: print end effector position (throttled to 1Hz)
                        if not hasattr(state_broadcast_loop, '_last_fk_print') or \
                           time.time() - state_broadcast_loop._last_fk_print > 1.0:
                            state_broadcast_loop._last_fk_print = time.time()
                            print(f"[FK] EE position: [{fk['position'][0]:.4f}, {fk['position'][1]:.4f}, {fk['position'][2]:.4f}]")
                except Exception as fk_error:
                    pass  # FK calculation failed, keep previous values

            else:
                # Demo mode: smoothly interpolate to target
                with target_lock:
                    targets = target_positions.copy()

                for i in range(len(current_positions)):
                    diff = targets[i] - current_positions[i]
                    current_positions[i] += diff * 0.1  # Smooth interpolation

                # Demo mode FK: use robot if available, otherwise skip
                # Note: Pinocchio FK already applies tool_offset=[0,0,0.14] by default
                if robot is not None:
                    try:
                        fk = robot.forward_kinematics(np.array(current_positions))
                        if fk:
                            rotation = fk['rotation']
                            current_fk['position'] = fk['position'] if isinstance(fk['position'], list) else fk['position'].tolist()
                            current_fk['rotation'] = rotation.tolist() if hasattr(rotation, 'tolist') else [list(row) for row in rotation]
                            current_fk['euler'] = rotation_matrix_to_euler(rotation)
                    except Exception:
                        pass

            # Broadcast to all connected clients
            if connected_clients:
                with target_lock:
                    mode = control_mode
                    imp_target = impedance_target.tolist()

                socketio.emit('robot_state', {
                    'positions': current_positions,
                    'velocities': current_velocities,
                    'torques': current_torques,
                    'target_positions': target_positions,
                    'control_mode': mode,
                    'impedance_target': imp_target,
                    'forward_kinematics': current_fk,
                    'timestamp': time.time()
                })

        except Exception as e:
            print(f"Broadcast loop error: {e}")

        elapsed = time.time() - loop_start
        sleep_time = dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


def start_loops():
    """Start control and broadcast loops"""
    global loop_running
    loop_running = True

    # Control loop (high frequency)
    control_thread = threading.Thread(target=control_loop, daemon=True)
    control_thread.start()

    # Broadcast loop (medium frequency)
    broadcast_thread = threading.Thread(target=state_broadcast_loop, daemon=True)
    broadcast_thread.start()

    print(f"Control loop started at {CONTROL_FREQ} Hz")
    print(f"Broadcast loop started at {BROADCAST_FREQ} Hz")


# ============== Static Files - arm_description ==============
ARM_DESCRIPTION_PATH = os.path.join(os.path.dirname(__file__), '..', 'arm_description')


@app.route('/arm_description/<path:filename>')
def serve_arm_description(filename):
    """Serve files from arm_description folder"""
    return send_from_directory(ARM_DESCRIPTION_PATH, filename)


# ============== REST API Routes ==============

@app.route('/')
def index():
    """Serve frontend"""
    return send_from_directory(app.static_folder, 'index.html')


@app.route('/api/config')
def get_config():
    """Get robot configuration for frontend"""
    return jsonify({
        "robot_name": robot_config['robot']['name'] if robot_config else "Unknown",
        "joints": JOINT_CONFIG,
        "urdf_path": URDF_PATH,
        "demo_mode": demo_mode,
        "control_freq": CONTROL_FREQ,
        "connected": robot is not None or demo_mode,
        "end_effector_offset": END_EFFECTOR_OFFSET
    })


@app.route('/api/arm_description_files')
def get_arm_description_files():
    """Get list of files in arm_description folder for auto-loading"""
    files = {}

    def scan_directory(path, prefix=''):
        """Recursively scan directory and build file list"""
        for entry in os.scandir(path):
            rel_path = os.path.join(prefix, entry.name) if prefix else entry.name
            if entry.is_file():
                files[rel_path] = f'/arm_description/{rel_path}'
            elif entry.is_dir():
                scan_directory(entry.path, rel_path)

    try:
        scan_directory(ARM_DESCRIPTION_PATH)
        return jsonify({
            "success": True,
            "files": files,
            "base_url": "/arm_description"
        })
    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500


@app.route('/api/status')
def get_status():
    """Get current robot status (REST fallback)"""
    return jsonify({
        "connected": robot is not None or demo_mode,
        "demo_mode": demo_mode,
        "positions": current_positions,
        "velocities": current_velocities,
        "torques": current_torques,
        "target_positions": target_positions,
        "target_velocity": target_velocity
    })


@app.route('/api/move_joint', methods=['POST'])
def move_joint():
    """Move a single joint"""
    global target_positions

    data = request.json
    joint_index = data.get('joint')
    position = data.get('position')

    if joint_index is not None and position is not None:
        # Clamp to joint limits
        if JOINT_CONFIG and joint_index < len(JOINT_CONFIG):
            jc = JOINT_CONFIG[joint_index]
            position = max(jc['min'], min(jc['max'], position))

        with target_lock:
            target_positions[joint_index] = position

    return jsonify({"success": True})


@app.route('/api/move', methods=['POST'])
def move_all():
    """Move all joints"""
    global target_positions, target_velocity

    data = request.json

    with target_lock:
        if 'positions' in data:
            positions = data['positions']
            # Clamp to limits
            for i, pos in enumerate(positions):
                if JOINT_CONFIG and i < len(JOINT_CONFIG):
                    jc = JOINT_CONFIG[i]
                    positions[i] = max(jc['min'], min(jc['max'], pos))
            target_positions[:] = positions

        if 'velocity' in data:
            target_velocity = data['velocity']

    return jsonify({"success": True})


@app.route('/api/home', methods=['POST'])
def go_home():
    """Move to home position"""
    global target_positions

    with target_lock:
        target_positions[:] = [0.0] * len(target_positions)

    return jsonify({"success": True})


@app.route('/api/stop', methods=['POST'])
def stop():
    """Stop at current position"""
    global target_positions

    with target_lock:
        target_positions[:] = current_positions.copy()

    return jsonify({"success": True})


@app.route('/api/set_zero', methods=['POST'])
def set_zero():
    """Reset encoder positions to zero (set current position as zero reference)"""
    global robot, current_positions, target_positions

    if robot is None:
        return jsonify({"success": False, "error": "Robot not connected"}), 400

    try:
        # Call the robot's set_reset_zero method
        robot.set_reset_zero()
        robot.motor_send_cmd()

        # Reset our tracked positions to zero
        with target_lock:
            current_positions[:] = [0.0] * len(current_positions)
            target_positions[:] = [0.0] * len(target_positions)

        print("[Set Zero] Encoder positions reset to zero")

        # Broadcast the update to all clients
        socketio.emit('joint_positions', {
            'positions': [0.0] * len(current_positions),
            'velocities': [0.0] * len(current_positions),
            'torques': [0.0] * len(current_positions)
        })

        return jsonify({"success": True, "message": "Encoder positions reset to zero"})

    except Exception as e:
        print(f"[Set Zero] Error: {e}")
        return jsonify({"success": False, "error": str(e)}), 500


@app.route('/api/set_velocity', methods=['POST'])
def set_velocity():
    """Set movement velocity"""
    global target_velocity

    with target_lock:
        target_velocity = request.json.get('velocity', 0.5)

    return jsonify({"success": True, "velocity": target_velocity})


@app.route('/api/set_mode', methods=['POST'])
def set_mode():
    """Set control mode: 'position', 'gravity_comp', 'impedance'"""
    global control_mode, impedance_target

    data = request.json
    mode = data.get('mode', 'position')

    if mode not in ['position', 'gravity_comp', 'impedance']:
        return jsonify({"success": False, "error": "Invalid mode"}), 400

    with target_lock:
        control_mode = mode

        # When switching to impedance mode, set current position as target
        if mode == 'impedance':
            impedance_target = np.array(current_positions)

    print(f"Control mode changed to: {mode}")
    return jsonify({"success": True, "mode": mode})


@app.route('/api/get_mode', methods=['GET'])
def get_mode():
    """Get current control mode and parameters"""
    return jsonify({
        "mode": control_mode,
        "impedance": {
            "K": impedance_K.tolist(),
            "B": impedance_B.tolist(),
            "target": impedance_target.tolist()
        },
        "gravity_comp": {
            "gain": gravity_gain.tolist(),
            "offset": joint_offset.tolist()
        }
    })


@app.route('/api/set_impedance_params', methods=['POST'])
def set_impedance_params():
    """Set impedance control parameters"""
    global impedance_K, impedance_B, impedance_target

    data = request.json

    with target_lock:
        if 'K' in data:
            impedance_K = np.array(data['K'])
        if 'B' in data:
            impedance_B = np.array(data['B'])
        if 'target' in data:
            impedance_target = np.array(data['target'])

    return jsonify({
        "success": True,
        "K": impedance_K.tolist(),
        "B": impedance_B.tolist(),
        "target": impedance_target.tolist()
    })


@app.route('/api/set_impedance_target', methods=['POST'])
def set_impedance_target():
    """Set impedance control target position"""
    global impedance_target

    data = request.json

    with target_lock:
        if 'target' in data:
            impedance_target = np.array(data['target'])
        elif 'joint' in data and 'position' in data:
            # Set single joint target
            joint_index = data['joint']
            impedance_target[joint_index] = data['position']

    return jsonify({"success": True, "target": impedance_target.tolist()})


# ============== Waypoint API Routes ==============

@app.route('/api/waypoints', methods=['GET'])
def get_waypoints():
    """Get all waypoints"""
    return jsonify({
        "success": True,
        "waypoints": waypoints,
        "max_waypoints": MAX_WAYPOINTS,
        "trajectory_running": trajectory_running
    })


@app.route('/api/waypoints/add', methods=['POST'])
def add_waypoint():
    """Add current position as a new waypoint"""
    global waypoints

    if len(waypoints) >= MAX_WAYPOINTS:
        return jsonify({"success": False, "error": f"Maximum {MAX_WAYPOINTS} waypoints allowed"}), 400

    data = request.json

    if 'positions' in data:
        # Use provided positions
        positions = data['positions']
    else:
        # Use current robot positions
        positions = current_positions.copy()

    duration = data.get('duration', 1.0)

    waypoint = {
        'positions': positions,
        'duration': duration,
        'index': len(waypoints)
    }
    waypoints.append(waypoint)

    print(f"Added waypoint {len(waypoints)}: {positions}")
    return jsonify({"success": True, "waypoint": waypoint, "total": len(waypoints)})


@app.route('/api/waypoints/update', methods=['POST'])
def update_waypoint():
    """Update a waypoint"""
    global waypoints

    data = request.json
    index = data.get('index')

    if index is None or index < 0 or index >= len(waypoints):
        return jsonify({"success": False, "error": "Invalid waypoint index"}), 400

    if 'positions' in data:
        waypoints[index]['positions'] = data['positions']
    if 'duration' in data:
        waypoints[index]['duration'] = data['duration']

    return jsonify({"success": True, "waypoint": waypoints[index]})


@app.route('/api/waypoints/delete', methods=['POST'])
def delete_waypoint():
    """Delete a waypoint"""
    global waypoints

    data = request.json
    index = data.get('index')

    if index is None or index < 0 or index >= len(waypoints):
        return jsonify({"success": False, "error": "Invalid waypoint index"}), 400

    deleted = waypoints.pop(index)

    # Update indices
    for i, wp in enumerate(waypoints):
        wp['index'] = i

    return jsonify({"success": True, "deleted": deleted, "total": len(waypoints)})


@app.route('/api/waypoints/clear', methods=['POST'])
def clear_waypoints():
    """Clear all waypoints"""
    global waypoints
    waypoints = []
    return jsonify({"success": True})


@app.route('/api/waypoints/go_to', methods=['POST'])
def go_to_waypoint():
    """Move robot to a specific waypoint"""
    global target_positions

    data = request.json
    index = data.get('index')

    if index is None or index < 0 or index >= len(waypoints):
        return jsonify({"success": False, "error": "Invalid waypoint index"}), 400

    positions = waypoints[index]['positions']

    with target_lock:
        target_positions[:] = positions

    return jsonify({"success": True, "target": positions})


@app.route('/api/trajectory/run', methods=['POST'])
def run_trajectory():
    """Execute trajectory through all waypoints"""
    global trajectory_running

    if trajectory_running:
        return jsonify({"success": False, "error": "Trajectory already running"}), 400

    if len(waypoints) < 2:
        return jsonify({"success": False, "error": "Need at least 2 waypoints"}), 400

    data = request.json
    control_rate = data.get('control_rate', 100)

    # Build waypoint list and durations
    waypoint_list = [wp['positions'] for wp in waypoints]
    durations = [wp['duration'] for wp in waypoints[:-1]]  # n-1 durations for n waypoints

    # Start trajectory in separate thread
    traj_thread = threading.Thread(
        target=execute_trajectory_thread,
        args=(waypoint_list, durations, control_rate),
        daemon=True
    )
    traj_thread.start()

    return jsonify({"success": True, "message": "Trajectory started"})


@app.route('/api/trajectory/stop', methods=['POST'])
def stop_trajectory():
    """Stop running trajectory"""
    global trajectory_running

    trajectory_running = False
    return jsonify({"success": True})


@app.route('/api/trajectory/status', methods=['GET'])
def trajectory_status():
    """Get trajectory execution status"""
    return jsonify({
        "running": trajectory_running,
        "progress": trajectory_progress
    })


# ============== WebSocket Events ==============

@socketio.on('connect')
def handle_connect():
    """Handle new WebSocket connection"""
    connected_clients.add(request.sid)
    print(f"Client connected: {request.sid} (total: {len(connected_clients)})")

    # Send initial config
    emit('config', {
        "robot_name": robot_config['robot']['name'] if robot_config else "Unknown",
        "joints": JOINT_CONFIG,
        "demo_mode": demo_mode,
        "connected": robot is not None or demo_mode,
        "control_mode": control_mode,
        "impedance": {
            "K": impedance_K.tolist(),
            "B": impedance_B.tolist(),
            "target": impedance_target.tolist()
        },
        "end_effector_offset": END_EFFECTOR_OFFSET
    })


@socketio.on('disconnect')
def handle_disconnect():
    """Handle WebSocket disconnection"""
    connected_clients.discard(request.sid)
    print(f"Client disconnected: {request.sid} (total: {len(connected_clients)})")


@socketio.on('move_joint')
def handle_move_joint(data):
    """Handle joint movement command via WebSocket"""
    global target_positions

    joint_index = data.get('joint')
    position = data.get('position')

    if joint_index is not None and position is not None:
        if JOINT_CONFIG and joint_index < len(JOINT_CONFIG):
            jc = JOINT_CONFIG[joint_index]
            position = max(jc['min'], min(jc['max'], position))

        with target_lock:
            target_positions[joint_index] = position


@socketio.on('move_all')
def handle_move_all(data):
    """Handle all joints movement via WebSocket"""
    global target_positions, target_velocity

    with target_lock:
        if 'positions' in data:
            positions = data['positions']
            for i, pos in enumerate(positions):
                if JOINT_CONFIG and i < len(JOINT_CONFIG):
                    jc = JOINT_CONFIG[i]
                    positions[i] = max(jc['min'], min(jc['max'], pos))
            target_positions[:] = positions

        if 'velocity' in data:
            target_velocity = data['velocity']


@socketio.on('home')
def handle_home():
    """Handle home command via WebSocket"""
    global target_positions

    with target_lock:
        target_positions[:] = [0.0] * len(target_positions)


@socketio.on('stop')
def handle_stop():
    """Handle stop command via WebSocket"""
    global target_positions

    with target_lock:
        target_positions[:] = current_positions.copy()


@socketio.on('set_zero')
def handle_set_zero():
    """Handle set zero command via WebSocket - reset encoder positions to zero"""
    global robot, current_positions, target_positions

    if robot is None:
        emit('set_zero_result', {'success': False, 'error': 'Robot not connected'})
        return

    try:
        # Call the robot's set_reset_zero method
        robot.set_reset_zero()
        robot.motor_send_cmd()

        # Reset our tracked positions to zero
        with target_lock:
            current_positions[:] = [0.0] * len(current_positions)
            target_positions[:] = [0.0] * len(target_positions)

        print("[Set Zero] Encoder positions reset to zero")

        # Broadcast the update to all clients
        socketio.emit('joint_positions', {
            'positions': [0.0] * len(current_positions),
            'velocities': [0.0] * len(current_positions),
            'torques': [0.0] * len(current_positions)
        })

        emit('set_zero_result', {'success': True, 'message': 'Encoder positions reset to zero'})

    except Exception as e:
        print(f"[Set Zero] Error: {e}")
        emit('set_zero_result', {'success': False, 'error': str(e)})


@socketio.on('set_mode')
def handle_set_mode(data):
    """Handle control mode change via WebSocket"""
    global control_mode, impedance_target

    mode = data.get('mode', 'position')

    if mode in ['position', 'gravity_comp', 'impedance']:
        with target_lock:
            control_mode = mode
            # When switching to impedance mode, set current position as target
            if mode == 'impedance':
                impedance_target = np.array(current_positions)

        print(f"Control mode changed to: {mode}")

        # Broadcast mode change to all clients
        socketio.emit('mode_changed', {'mode': mode})


@socketio.on('set_impedance_target')
def handle_set_impedance_target(data):
    """Handle impedance target change via WebSocket"""
    global impedance_target

    with target_lock:
        if 'target' in data:
            impedance_target = np.array(data['target'])
        elif 'joint' in data and 'position' in data:
            joint_index = data['joint']
            impedance_target[joint_index] = data['position']


@socketio.on('set_impedance_params')
def handle_set_impedance_params(data):
    """Handle impedance parameter change via WebSocket"""
    global impedance_K, impedance_B

    with target_lock:
        if 'K' in data:
            impedance_K = np.array(data['K'])
        if 'B' in data:
            impedance_B = np.array(data['B'])


@socketio.on('add_waypoint')
def handle_add_waypoint(data):
    """Add waypoint via WebSocket"""
    global waypoints

    if len(waypoints) >= MAX_WAYPOINTS:
        emit('waypoint_error', {'error': f'Maximum {MAX_WAYPOINTS} waypoints allowed'})
        return

    if 'positions' in data:
        positions = data['positions']
    else:
        positions = current_positions.copy()

    duration = data.get('duration', 1.0)

    waypoint = {
        'positions': positions,
        'duration': duration,
        'index': len(waypoints)
    }
    waypoints.append(waypoint)

    # Broadcast to all clients
    socketio.emit('waypoints_updated', {'waypoints': waypoints})


@socketio.on('delete_waypoint')
def handle_delete_waypoint(data):
    """Delete waypoint via WebSocket"""
    global waypoints

    index = data.get('index')
    if index is None or index < 0 or index >= len(waypoints):
        emit('waypoint_error', {'error': 'Invalid waypoint index'})
        return

    waypoints.pop(index)

    # Update indices
    for i, wp in enumerate(waypoints):
        wp['index'] = i

    socketio.emit('waypoints_updated', {'waypoints': waypoints})


@socketio.on('clear_waypoints')
def handle_clear_waypoints():
    """Clear all waypoints via WebSocket"""
    global waypoints
    waypoints = []
    socketio.emit('waypoints_updated', {'waypoints': waypoints})


@socketio.on('update_waypoint_duration')
def handle_update_waypoint_duration(data):
    """Update waypoint duration via WebSocket"""
    global waypoints

    index = data.get('index')
    duration = data.get('duration')

    if index is None or index < 0 or index >= len(waypoints):
        return

    if duration is not None:
        waypoints[index]['duration'] = duration
        socketio.emit('waypoints_updated', {'waypoints': waypoints})


@socketio.on('run_trajectory')
def handle_run_trajectory(data):
    """Run trajectory via WebSocket"""
    global trajectory_running

    if trajectory_running:
        emit('trajectory_error', {'error': 'Trajectory already running'})
        return

    if len(waypoints) < 2:
        emit('trajectory_error', {'error': 'Need at least 2 waypoints'})
        return

    control_rate = data.get('control_rate', 100)

    waypoint_list = [wp['positions'] for wp in waypoints]
    durations = [wp['duration'] for wp in waypoints[:-1]]

    traj_thread = threading.Thread(
        target=execute_trajectory_thread,
        args=(waypoint_list, durations, control_rate),
        daemon=True
    )
    traj_thread.start()


@socketio.on('stop_trajectory')
def handle_stop_trajectory():
    """Stop trajectory via WebSocket"""
    global trajectory_running
    trajectory_running = False


@socketio.on('go_to_waypoint')
def handle_go_to_waypoint(data):
    """Move to waypoint via WebSocket"""
    global target_positions

    index = data.get('index')
    if index is None or index < 0 or index >= len(waypoints):
        return

    positions = waypoints[index]['positions']

    with target_lock:
        target_positions[:] = positions


# ============== Camera WebSocket Events ==============

def camera_broadcast_loop():
    """Broadcast camera image (mono or depth) to connected clients."""
    global camera_streaming
    import cv2

    dt = 1.0 / CAMERA_BROADCAST_FREQ

    while camera_streaming and camera_streamer is not None:
        loop_start = time.time()

        try:
            data = camera_streamer.get_latest_data()

            if data['image'] is not None:
                image = data['image']

                # Encode as JPEG for efficient transmission
                _, jpeg_buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                jpeg_base64 = base64.b64encode(jpeg_buffer).decode('utf-8')

                # Emit image data
                emit_data = {
                    'image': jpeg_base64,
                    'width': image.shape[1],
                    'height': image.shape[0],
                    'mode': data['mode'],
                    'fps': data['fps'],
                    'latency_ms': data['latency_ms'],
                    'frame_count': data['frame_count']
                }

                socketio.emit('camera_frame', emit_data)

        except Exception as e:
            print(f"Camera broadcast error: {e}")

        # Maintain target frequency
        elapsed = time.time() - loop_start
        sleep_time = dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)


@socketio.on('camera_start')
def handle_camera_start(data=None):
    """Start camera streaming."""
    global camera_streaming, camera_streamer

    if camera_streaming:
        emit('camera_status', {'streaming': True, 'message': 'Already streaming'})
        return

    # Initialize camera if needed
    if camera_streamer is None:
        resolution = data.get('resolution', (640, 400)) if data else (640, 400)
        fps = data.get('fps', 60) if data else 60  # Default 60fps for smooth video
        downsample = data.get('downsample', 1) if data else 1
        camera_streamer = CameraStreamer(
            resolution=tuple(resolution),
            fps=fps,
            downsample=downsample
        )

    # Start the camera
    if camera_streamer.start():
        camera_streaming = True

        # Start broadcast loop in separate thread
        broadcast_thread = threading.Thread(
            target=camera_broadcast_loop,
            daemon=True
        )
        broadcast_thread.start()

        emit('camera_status', {
            'streaming': True,
            'config': camera_streamer.get_config(),
            'message': 'Camera started'
        })
        socketio.emit('camera_started', camera_streamer.get_config())
    else:
        emit('camera_status', {'streaming': False, 'error': 'Failed to start camera'})


@socketio.on('camera_stop')
def handle_camera_stop():
    """Stop camera streaming."""
    global camera_streaming

    camera_streaming = False
    if camera_streamer:
        camera_streamer.stop()

    emit('camera_status', {'streaming': False, 'message': 'Camera stopped'})
    socketio.emit('camera_stopped', {})


@socketio.on('camera_config')
def handle_camera_config():
    """Get camera configuration."""
    if camera_streamer:
        emit('camera_config_response', camera_streamer.get_config())
    else:
        emit('camera_config_response', {
            'available': False,
            'streaming': False
        })


@socketio.on('camera_mode')
def handle_camera_mode(data):
    """Set camera streaming mode (mono or depth)."""
    if not camera_streamer:
        emit('camera_mode_response', {'success': False, 'error': 'Camera not initialized'})
        return

    mode = data.get('mode', 'mono')
    if camera_streamer.set_mode(mode):
        emit('camera_mode_response', {'success': True, 'mode': mode})
        socketio.emit('camera_mode_changed', {'mode': mode})
    else:
        emit('camera_mode_response', {'success': False, 'error': f'Invalid mode: {mode}'})


@socketio.on('camera_depth_query')
def handle_camera_depth_query(data):
    """Query depth at a selected region and return 3D position."""
    global last_camera_position

    if not camera_streamer:
        emit('camera_depth_response', {'success': False, 'error': 'Camera not initialized'})
        return

    box = data.get('box')  # [x1, y1, x2, y2]
    if not box or len(box) != 4:
        emit('camera_depth_response', {'success': False, 'error': 'Invalid box coordinates'})
        return

    result = camera_streamer.get_depth_at_region(box)
    if result:
        # Store the camera position for calibration recording
        last_camera_position = [result['x'], result['y'], result['z']]

        # Apply cam2bot transformation if available
        cam2bot_position = apply_cam2bot_transform(last_camera_position)

        response = {
            'success': True,
            'position': {
                'x': result['x'],
                'y': result['y'],
                'z': result['z']
            },
            'depth_range': {
                'min': result['min_depth'],
                'max': result['max_depth'],
                'median': result['median_depth']
            },
            'box': result['box']
        }

        # Add cam2bot position if transformation is available
        if cam2bot_position:
            response['cam2bot_position'] = {
                'x': cam2bot_position[0],
                'y': cam2bot_position[1],
                'z': cam2bot_position[2]
            }

        emit('camera_depth_response', response)
    else:
        emit('camera_depth_response', {'success': False, 'error': 'Could not compute depth (no valid disparity in region)'})


# ============== Calibration Recording ==============

@socketio.on('record_calibration_pair')
def handle_record_calibration_pair(data=None):
    """Record a calibration pair (camera position + robot EE position)."""
    global calibration_pairs, last_camera_position, current_fk

    if last_camera_position is None:
        emit('calibration_response', {
            'success': False,
            'error': 'No camera position available. Please select a point in the camera view first.'
        })
        return

    # Get current robot end effector position
    robot_position = current_fk.get('position', None)
    if robot_position is None or robot_position == [0.0, 0.0, 0.0]:
        emit('calibration_response', {
            'success': False,
            'error': 'No robot position available. Please ensure robot is connected.'
        })
        return

    # Create calibration pair (convert numpy types to Python floats for proper YAML serialization)
    pair = {
        'camera_frame': {
            'x': float(last_camera_position[0]),
            'y': float(last_camera_position[1]),
            'z': float(last_camera_position[2])
        },
        'robot_frame': {
            'x': float(robot_position[0]),
            'y': float(robot_position[1]),
            'z': float(robot_position[2])
        }
    }

    calibration_pairs.append(pair)
    pair_count = len(calibration_pairs)

    print(f"[Calibration] Recorded pair #{pair_count}:")
    print(f"  Camera: [{pair['camera_frame']['x']:.4f}, {pair['camera_frame']['y']:.4f}, {pair['camera_frame']['z']:.4f}]")
    print(f"  Robot:  [{pair['robot_frame']['x']:.4f}, {pair['robot_frame']['y']:.4f}, {pair['robot_frame']['z']:.4f}]")

    emit('calibration_response', {
        'success': True,
        'message': f'Recorded calibration pair #{pair_count}',
        'pair': pair,
        'total_pairs': pair_count
    })

    # Broadcast updated pairs list to all clients
    socketio.emit('calibration_pairs_updated', {
        'total_pairs': pair_count,
        'pairs': calibration_pairs
    })


@socketio.on('clear_calibration_pairs')
def handle_clear_calibration_pairs(data=None):
    """Clear all recorded calibration pairs."""
    global calibration_pairs

    calibration_pairs = []
    print("[Calibration] Cleared all calibration pairs")

    emit('calibration_response', {
        'success': True,
        'message': 'Cleared all calibration pairs',
        'total_pairs': 0
    })

    socketio.emit('calibration_pairs_updated', {'total_pairs': 0, 'pairs': []})


@socketio.on('get_calibration_pairs')
def handle_get_calibration_pairs(data=None):
    """Get all recorded calibration pairs."""
    emit('calibration_pairs', {
        'success': True,
        'pairs': calibration_pairs,
        'total_pairs': len(calibration_pairs)
    })


@socketio.on('save_calibration_pairs')
def handle_save_calibration_pairs(data=None):
    """Save calibration pairs to a YAML file."""
    global calibration_pairs

    if len(calibration_pairs) == 0:
        emit('calibration_response', {
            'success': False,
            'error': 'No calibration pairs to save'
        })
        return

    # Default filename with timestamp
    from datetime import datetime
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = data.get('filename', f'calibration_pairs_{timestamp}.yaml') if data else f'calibration_pairs_{timestamp}.yaml'

    # Save to calibration directory
    calibration_dir = os.path.join(os.path.dirname(__file__), '..', 'calibration')
    os.makedirs(calibration_dir, exist_ok=True)
    filepath = os.path.join(calibration_dir, filename)

    # Prepare YAML data
    yaml_data = {
        'calibration_pairs': [],
        'metadata': {
            'timestamp': datetime.now().isoformat(),
            'total_pairs': len(calibration_pairs),
            'description': 'Camera-to-robot calibration data pairs'
        }
    }

    for i, pair in enumerate(calibration_pairs):
        yaml_data['calibration_pairs'].append({
            'pair_id': i + 1,
            'camera_frame': pair['camera_frame'],
            'robot_frame': pair['robot_frame']
        })

    try:
        with open(filepath, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False, sort_keys=False)

        print(f"[Calibration] Saved {len(calibration_pairs)} pairs to: {filepath}")

        emit('calibration_response', {
            'success': True,
            'message': f'Saved {len(calibration_pairs)} calibration pairs',
            'filepath': filepath,
            'filename': filename
        })
    except Exception as e:
        emit('calibration_response', {
            'success': False,
            'error': f'Failed to save: {str(e)}'
        })


# ============== Camera REST API ==============

@app.route('/api/camera/status', methods=['GET'])
def camera_status():
    """Get camera status."""
    if camera_streamer:
        return jsonify({
            'streaming': camera_streaming,
            'config': camera_streamer.get_config()
        })
    return jsonify({
        'streaming': False,
        'available': False
    })


@app.route('/api/camera/start', methods=['POST'])
def camera_start():
    """Start camera via REST API."""
    global camera_streaming, camera_streamer

    if camera_streaming:
        return jsonify({'success': True, 'message': 'Already streaming'})

    data = request.json or {}
    resolution = data.get('resolution', (640, 400))
    fps = data.get('fps', 60)
    downsample = data.get('downsample', 1)

    if camera_streamer is None:
        camera_streamer = CameraStreamer(
            resolution=tuple(resolution),
            fps=fps,
            downsample=downsample
        )

    if camera_streamer.start():
        camera_streaming = True

        broadcast_thread = threading.Thread(
            target=camera_broadcast_loop,
            daemon=True
        )
        broadcast_thread.start()

        return jsonify({
            'success': True,
            'config': camera_streamer.get_config()
        })

    return jsonify({'success': False, 'error': 'Failed to start camera'})


@app.route('/api/camera/stop', methods=['POST'])
def camera_stop():
    """Stop camera via REST API."""
    global camera_streaming

    camera_streaming = False
    if camera_streamer:
        camera_streamer.stop()

    return jsonify({'success': True})


# ============== Main ==============

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Digital Twin Backend Server')
    parser.add_argument('--config', '-c', type=str,
                        default=LOCAL_CONFIG_PATH,
                        help='Path to robot config YAML')
    parser.add_argument('--demo', action='store_true',
                        help='Run in demo mode without robot connection')
    parser.add_argument('--port', '-p', type=int, default=5000,
                        help='Server port (default: 5000)')
    args = parser.parse_args()

    demo_mode = args.demo

    print("=" * 50)
    print("Digital Twin Backend Server")
    print("=" * 50)

    # Load configuration
    config_path = args.config
    if not os.path.isabs(config_path):
        config_path = os.path.join(os.getcwd(), config_path)

    print(f"\n1. Loading configuration from: {config_path}")
    try:
        load_config(config_path)
    except Exception as e:
        print(f"Failed to load config: {e}")
        print("Using default joint configuration")
        JOINT_CONFIG = [
            {"name": f"Joint {i+1}", "index": i, "min": -3.14, "max": 3.14}
            for i in range(6)
        ]

    # Load cam2bot transformation
    print("\n2. Loading cam2bot transformation...")
    load_cam2bot_transform()

    # Initialize robot
    print("\n3. Initializing robot...")
    init_robot(config_path)

    # Start control loops
    print("\n4. Starting control loops...")
    start_loops()

    # Start server
    print(f"\n5. Starting server on port {args.port}...")
    print(f"\n   Backend API: http://localhost:{args.port}")
    print(f"   WebSocket:   ws://localhost:{args.port}")
    print("=" * 50)

    socketio.run(app, host='0.0.0.0', port=args.port, debug=False)

# Panthera Digital Twin - Robot Visualization & Control

A real-time digital twin interface that combines 3D URDF visualization with live robot connection and control for Panthera robotic arm.

![Panthera_test](https://github.com/user-attachments/assets/efe430dd-d56a-4751-a395-62b3c6650f50)

**Supported Control:** Position | Gravity Compensation | Impedance (PD + Gravity) | Set Encoder Zero | Add Waypoints | Trajectory Replay

## Architecture

Frontend adapted from: https://github.com/fan-ziqi/robot_viewer
Backend requires Panthera SDK: https://github.com/HighTorque-Robotics

```
Panthera_digital_twin/
├── backend/
│   ├── app.py              # Flask + WebSocket server
│   ├── camera/
│   │   ├── camera_streamer.py  # OAK-D stereo depth streaming
│   │   └── cam2bot.yaml        # Camera-to-robot transform
│   └── requirements.txt    # Python dependencies
├── frontend/
│   ├── src/
│   │   ├── main.js         # Application entry point
│   │   ├── robot/
│   │   │   └── RobotConnection.js  # WebSocket client
│   │   ├── ui/
│   │   │   ├── ConnectionUI.js     # Connection panel
│   │   │   ├── JointControlsUI.js  # Joint sliders & robot sync
│   │   │   ├── PanelManager.js     # Panel drag & resize
│   │   │   └── UIController.js     # Visualization & toolbar buttons
│   │   ├── renderer/       # Three.js scene management
│   │   └── adapters/       # URDF/MJCF parsing
│   ├── index.html          # Main HTML page
│   ├── package.json        # Node.js dependencies
│   └── vite.config.js      # Build configuration
├── calibration/
│   ├── calibrate_hand_eye.py   # Compute camera-to-robot transform
│   └── README.md               # Calibration instructions
├── robot_param/
│   └── xlb.yaml            # Robot configuration
├── arm_description/        # URDF and mesh files
│   ├── urdf/
│   └── meshes/
└── README.md
```

## Quick Start

### 1. Install Backend Dependencies

```bash
cd Panthera_digital_twin/backend
pip install -r requirements.txt
```

### 2. Install Frontend Dependencies

```bash
cd Panthera_digital_twin/frontend
npm install
# or with pnpm:
pnpm install
```

### 3. Start the Backend Server

**With real robot:**
```bash
cd Panthera_digital_twin/backend
python app.py --config ../robot_param/xlb.yaml
```

**Demo mode (no robot):**
```bash
cd Panthera_digital_twin/backend
python app.py --demo
```

The backend server will start on `http://localhost:5000`.

### 4. Start the Frontend Development Server

```bash
cd Panthera_digital_twin/frontend
npm run dev
```

The frontend will start on `http://localhost:3000` with hot-reload enabled.

### 5. Open the Application

Open your browser and navigate to `http://localhost:3000`.

## Usage

### Offline Mode (URDF Viewer)

1. The default URDF loads automatically from `arm_description/`
2. Select specific URDF file
3. Use joint sliders or drag digital twin links to manipulate the model

### Connected Mode (Real Robot)

1. Ensure the backend server is running with your robot configuration
2. Click the connection indicator in the top bar
3. Enter the server URL (default: `http://localhost:5000`)
4. Click "Connect"
5. Once connected:
   - Joint positions stream in real-time from the robot
   - Moving sliders sends commands to the robot
   - Use Home/Stop/Set Zero buttons for control
   - Switch between Position, Gravity Comp, and Impedance modes

### Control Modes

| Mode | Description |
|------|-------------|
| **Position** | Direct joint position control via sliders |
| **Gravity Comp** | Robot compensates for gravity, allows manual manipulation |
| **Impedance** | Spring-damper behavior around target position |

### Waypoints & Trajectory

1. Open the Waypoints panel from the toolbar
2. Move the robot to desired positions (under Gravity mode)
3. Click "Add Current Position" to save waypoints (max 6)
4. Adjust duration between waypoints (optional)
5. Switch back from gravity to position mode and go back to home position
6. Click "Run Trajectory" to execute smooth motion
7. Waypoints appear as glowing orange dragon ball markers in 3D view

## Eye-to-Hand Calibration

The system supports camera-to-robot calibration using a stereo depth camera (OAK-D).

### Overview

Record the robot end effector position in both coordinate systems and compute the rigid transformation:
```
P_robot = R @ P_camera + t
```

### Quick Calibration Workflow

1. **Record pairs**: Camera panel → Select mode → click on end effector → Record
2. **Compute transform**: `python calibration/calibrate_hand_eye.py calibration_pairs_*.yaml --save`
3. **Deploy**: Copy `transform_*.yaml` to `backend/camera/cam2bot.yaml`

After calibration, the **C2B** (Camera-to-Base) row shows transformed coordinates in Select mode.

See [calibration/README.md](calibration/README.md) for detailed instructions.

## Backend API

### REST Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/config` | GET | Get robot configuration |
| `/api/status` | GET | Get current robot state |
| `/api/fk` | GET | Get forward kinematics (end effector pose) |
| `/api/move_joint` | POST | Move single joint |
| `/api/move` | POST | Move all joints |
| `/api/home` | POST | Move to home position |
| `/api/stop` | POST | Stop at current position |
| `/api/set_zero` | POST | Reset encoder positions to zero |
| `/api/set_velocity` | POST | Set movement velocity |
| `/api/set_mode` | POST | Set control mode |
| `/api/waypoints` | GET | Get current waypoints |

### WebSocket Events

| Event | Direction | Description |
|-------|-----------|-------------|
| `connect` | Client → Server | Connection established |
| `config` | Server → Client | Robot configuration |
| `robot_state` | Server → Client | Real-time position updates (30Hz) |
| `move_joint` | Client → Server | Move single joint |
| `move_all` | Client → Server | Move all joints |
| `home` | Client → Server | Go to home position |
| `stop` | Client → Server | Stop movement |
| `set_zero` | Client → Server | Reset encoder zero position |
| `set_mode` | Client → Server | Change control mode |
| `mode_changed` | Server → Client | Mode change confirmation |
| `add_waypoint` | Client → Server | Add current position as waypoint |
| `delete_waypoint` | Client → Server | Remove a waypoint |
| `clear_waypoints` | Client → Server | Remove all waypoints |
| `go_to_waypoint` | Client → Server | Move to specific waypoint |
| `run_trajectory` | Client → Server | Start trajectory execution |
| `stop_trajectory` | Client → Server | Stop trajectory execution |
| `waypoints_updated` | Server → Client | Waypoints list changed |
| `trajectory_progress` | Server → Client | Execution progress (0-1) |
| `trajectory_complete` | Server → Client | Trajectory finished |

## Configuration

### Robot Configuration (YAML)

```yaml
robot:
  name: "Panthera Arm"
  joint_limits:
    lower: [-3.14, 0.0, 0.0, -2.0, -3.14, -3.14]
    upper: [3.14, 3.5, 4.0, 1.57, 3.14, 3.14]

urdf:
  file_path: "../arm_description/urdf/robot.urdf"

kinematics:
  joint_names: ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
```

### Control Parameters

In `backend/app.py`:

```python
CONTROL_FREQ = 200         # Hz - control loop frequency
BROADCAST_FREQ = 30        # Hz - WebSocket broadcast frequency
END_EFFECTOR_OFFSET = 0.07 # meters - offset from Link_6 to end effector location (for forward kinematics)

# Gravity compensation
gravity_gain = [1.0, 1.0, 1.0, 1.2, 1.0, 1.0]
tau_limit = [10.0, 10.0, 10.0, 10.0, 10.0, 3.7]

# Impedance control
impedance_K = [5.0, 5.0, 5.0, 5.0, 5.0, 1.0]  # Stiffness
impedance_B = [0.5, 0.5, 0.5, 0.5, 0.5, 0.1]  # Damping
```

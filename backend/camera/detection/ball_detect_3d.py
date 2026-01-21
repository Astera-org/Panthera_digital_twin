#!/usr/bin/env python3
"""
Real-time ball detection with 3D position in robot frame.

Combines:
- YOLO for ball bounding box detection
- HoughCircles for precise circle center localization
- Stereo depth for 3D position (p_cam)
- cam2bot transformation for robot frame (p_robot)

Usage:
    python ball_detect_3d.py
    python ball_detect_3d.py --model path/to/model.pt --fps 60
"""

import argparse
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import depthai as dai
import yaml
from ultralytics import YOLO


# ============== KALMAN FILTER FOR 3D TRACKING ==============
class BallTracker3D:
    """
    Kalman filter for smoothing 3D ball position.

    State: [x, y, z, vx, vy, vz]
    Measurement: [x, y, z]
    Model: Constant velocity (no physics assumptions)
    """

    def __init__(self, dt=1/60, process_noise=1.0, measurement_noise=0.01):
        """
        Args:
            dt: Time step (1/fps)
            process_noise: How much velocity can change per step (higher = more responsive)
            measurement_noise: Sensor noise in meters (depth noise ~1cm = 0.01)
        """
        self.dt = dt

        # State: [x, y, z, vx, vy, vz]
        self.state = np.zeros(6)

        # State transition matrix (constant velocity model)
        # x_new = x + vx*dt, vx_new = vx
        self.F = np.array([
            [1, 0, 0, dt, 0,  0],
            [0, 1, 0, 0,  dt, 0],
            [0, 0, 1, 0,  0,  dt],
            [0, 0, 0, 1,  0,  0],
            [0, 0, 0, 0,  1,  0],
            [0, 0, 0, 0,  0,  1],
        ], dtype=np.float64)

        # Measurement matrix (we only observe position)
        self.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
        ], dtype=np.float64)

        # Process noise covariance (how much we expect state to change)
        # Higher = more responsive to changes, less smoothing
        q_pos = process_noise * dt**2  # Position uncertainty
        q_vel = process_noise          # Velocity uncertainty
        self.Q = np.diag([q_pos, q_pos, q_pos, q_vel, q_vel, q_vel])

        # Measurement noise covariance (sensor noise)
        self.R = np.diag([measurement_noise, measurement_noise, measurement_noise])

        # State covariance (uncertainty in state estimate)
        self.P = np.eye(6) * 1.0  # Initial uncertainty

        # Track initialization state
        self.initialized = False
        self.frames_since_detection = 0
        self.max_frames_without_detection = 10

    def predict(self):
        """Predict next state (call every frame, even without detection)."""
        if not self.initialized:
            return None

        # State prediction: x = F @ x
        self.state = self.F @ self.state

        # Covariance prediction: P = F @ P @ F.T + Q
        self.P = self.F @ self.P @ self.F.T + self.Q

        self.frames_since_detection += 1

        # Reset if too long without detection
        if self.frames_since_detection > self.max_frames_without_detection:
            self.initialized = False
            return None

        return self.state[:3].copy()  # Return predicted position

    def update(self, measurement):
        """
        Update state with new measurement.

        Args:
            measurement: [x, y, z] in meters

        Returns:
            Smoothed [x, y, z] position
        """
        z = np.array(measurement)

        if not self.initialized:
            # First measurement: initialize state
            self.state[:3] = z
            self.state[3:] = 0  # Zero velocity
            self.P = np.eye(6) * 0.1
            self.initialized = True
            self.frames_since_detection = 0
            return z.copy()

        # Kalman gain: K = P @ H.T @ (H @ P @ H.T + R)^-1
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # State update: x = x + K @ (z - H @ x)
        innovation = z - self.H @ self.state
        self.state = self.state + K @ innovation

        # Covariance update: P = (I - K @ H) @ P
        I = np.eye(6)
        self.P = (I - K @ self.H) @ self.P

        self.frames_since_detection = 0

        return self.state[:3].copy()

    def get_velocity(self):
        """Get estimated velocity [vx, vy, vz] in m/s."""
        if not self.initialized:
            return None
        return self.state[3:].copy()

    def reset(self):
        """Reset tracker state."""
        self.state = np.zeros(6)
        self.P = np.eye(6) * 1.0
        self.initialized = False
        self.frames_since_detection = 0


# ============== SIMPLE MOVING AVERAGE ==============
def moving_average(trajectory, window=3):
    """
    Apply simple moving average to trajectory.

    Args:
        trajectory: List of [x, y, z] positions
        window: Number of points to average (default 3)

    Returns:
        Smoothed trajectory (same length, edges use smaller window)
    """
    if len(trajectory) < 2:
        return list(trajectory)

    smoothed = []
    n = len(trajectory)

    for i in range(n):
        # Adaptive window at edges
        start = max(0, i - window + 1)
        end = i + 1
        points = trajectory[start:end]

        # Average
        avg = [
            sum(p[0] for p in points) / len(points),
            sum(p[1] for p in points) / len(points),
            sum(p[2] for p in points) / len(points),
        ]
        smoothed.append(avg)

    return smoothed


# ============== TRAJECTORY VISUALIZATION ==============
def get_trajectory_color(index, total):
    """
    Get color for trajectory point (cool to warm: blue -> red).

    Args:
        index: Point index (0 = oldest)
        total: Total number of points

    Returns:
        BGR color tuple
    """
    if total <= 1:
        return (0, 0, 255)  # Red for single point

    # Normalize to 0-1 (0 = oldest/cool, 1 = newest/warm)
    t = index / (total - 1)

    # Blue (cool) -> Cyan -> Green -> Yellow -> Red (warm)
    # Using HSV-like interpolation in BGR
    if t < 0.25:
        # Blue to Cyan
        r, g, b = 0, int(255 * t * 4), 255
    elif t < 0.5:
        # Cyan to Green
        r, g, b = 0, 255, int(255 * (1 - (t - 0.25) * 4))
    elif t < 0.75:
        # Green to Yellow
        r, g, b = int(255 * (t - 0.5) * 4), 255, 0
    else:
        # Yellow to Red
        r, g, b = 255, int(255 * (1 - (t - 0.75) * 4)), 0

    return (b, g, r)  # BGR format


def draw_trajectory(frame, trajectory, intrinsics):
    """
    Draw 2D trajectory on frame from 3D points.

    Args:
        frame: BGR image to draw on
        trajectory: List of [x, y, z] positions in camera frame (meters)
        intrinsics: 3x3 camera intrinsic matrix
    """
    if len(trajectory) < 2:
        return

    fx = intrinsics[0, 0]
    fy = intrinsics[1, 1]
    cx = intrinsics[0, 2]
    cy = intrinsics[1, 2]

    # Project 3D points to 2D
    points_2d = []
    for p in trajectory:
        if p[2] > 0:  # Valid depth
            u = int(p[0] * fx / p[2] + cx)
            v = int(p[1] * fy / p[2] + cy)
            points_2d.append((u, v))
        else:
            points_2d.append(None)

    # Draw trajectory lines and points
    total = len(points_2d)
    for i in range(total):
        if points_2d[i] is None:
            continue

        color = get_trajectory_color(i, total)

        # Draw point (larger for newer points)
        radius = max(2, int(3 * (i + 1) / total))
        cv2.circle(frame, points_2d[i], radius, color, -1)

        # Draw line to next point
        if i < total - 1 and points_2d[i + 1] is not None:
            thickness = max(1, int(2 * (i + 1) / total))
            cv2.line(frame, points_2d[i], points_2d[i + 1], color, thickness)


# ============== CAM2BOT TRANSFORMATION ==============
cam2bot_R = None  # 3x3 rotation matrix
cam2bot_t = None  # 3x1 translation vector


def load_cam2bot_transform(yaml_path=None):
    """Load camera-to-robot transformation from yaml file."""
    global cam2bot_R, cam2bot_t

    if yaml_path is None:
        yaml_path = Path(__file__).parent.parent / "cam2bot.yaml"

    if not yaml_path.exists():
        print(f"[Cam2Bot] Transform file not found: {yaml_path}")
        return False

    try:
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        transform = data.get('camera_to_robot_transform', {})
        cam2bot_R = np.array(transform.get('rotation_matrix', []))
        cam2bot_t = np.array(transform.get('translation', []))

        if cam2bot_R.shape != (3, 3) or cam2bot_t.shape != (3,):
            print("[Cam2Bot] Invalid transform dimensions")
            cam2bot_R = None
            cam2bot_t = None
            return False

        print(f"[Cam2Bot] Transform loaded from {yaml_path}")
        print(f"  Translation: [{cam2bot_t[0]:.4f}, {cam2bot_t[1]:.4f}, {cam2bot_t[2]:.4f}]")
        return True

    except Exception as e:
        print(f"[Cam2Bot] Failed to load transform: {e}")
        return False


def apply_cam2bot_transform(p_cam):
    """
    Transform point from camera frame to robot base frame.

    p_robot = R @ p_cam + t

    Args:
        p_cam: [x, y, z] in camera frame (meters)

    Returns:
        [x, y, z] in robot base frame (meters), or None if transform not loaded
    """
    global cam2bot_R, cam2bot_t

    if cam2bot_R is None or cam2bot_t is None:
        return None

    p_cam = np.array(p_cam)
    p_robot = cam2bot_R @ p_cam + cam2bot_t
    return p_robot.tolist()


# ============== CIRCLE DETECTION ==============
def detect_circle_in_bbox(gray_frame, bbox):
    """
    Detect circle within bounding box using HoughCircles.

    Args:
        gray_frame: Grayscale image
        bbox: (x1, y1, x2, y2) bounding box coordinates

    Returns:
        (cx, cy, radius) if circle found, None otherwise
    """
    x1, y1, x2, y2 = bbox
    h, w = gray_frame.shape[:2]

    # Clip to image bounds
    x1 = max(0, x1)
    y1 = max(0, y1)
    x2 = min(w, x2)
    y2 = min(h, y2)

    bbox_w = x2 - x1
    bbox_h = y2 - y1

    # Crop region of interest
    roi = gray_frame[y1:y2, x1:x2]
    if roi.size == 0 or bbox_w < 6 or bbox_h < 6:
        return None

    # Radius must fit inside bbox
    min_radius = max(2, min(bbox_w, bbox_h) // 6)
    max_radius = min(bbox_w, bbox_h) // 2

    # Apply Gaussian blur to reduce noise
    blurred = cv2.GaussianBlur(roi, (5, 5), 1.5)

    # Detect circles using Hough transform
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=min(bbox_w, bbox_h),
        param1=50,
        param2=25,
        minRadius=min_radius,
        maxRadius=max_radius
    )

    if circles is not None:
        circle = circles[0, 0]
        cx, cy, radius = circle
        # Convert back to original image coordinates
        cx = int(cx + x1)
        cy = int(cy + y1)
        radius = int(radius)
        radius = min(radius, (x2 - x1) // 2, (y2 - y1) // 2)
        return (cx, cy, radius)

    # Fallback: use bbox center
    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2
    radius = min(bbox_w, bbox_h) // 2
    return (cx, cy, radius)


# ============== DEPTH CALCULATION ==============
def get_depth_at_circle(disparity, cx, cy, radius, intrinsics, baseline_mm, subpixel_bits=5):
    """
    Calculate 3D position (p_cam) for a detected circle.

    Uses median depth within a region around the circle center.

    Args:
        disparity: Raw disparity map (subpixel values)
        cx, cy: Circle center in image coordinates
        radius: Circle radius in pixels
        intrinsics: 3x3 camera intrinsic matrix
        baseline_mm: Stereo baseline in mm
        subpixel_bits: Number of fractional bits (default 5)

    Returns:
        Dict with x, y, z in camera frame (meters), or None if invalid
    """
    h, w = disparity.shape[:2]

    # Use a small region around center for depth estimation
    sample_radius = max(3, radius // 3)
    x1 = max(0, cx - sample_radius)
    y1 = max(0, cy - sample_radius)
    x2 = min(w, cx + sample_radius)
    y2 = min(h, cy + sample_radius)

    if x2 <= x1 or y2 <= y1:
        return None

    roi = disparity[y1:y2, x1:x2].astype(np.float32)
    valid_mask = roi > 0

    if not np.any(valid_mask):
        return None

    # Extract intrinsic parameters
    fx = intrinsics[0, 0]
    fy = intrinsics[1, 1]
    cx_intrinsic = intrinsics[0, 2]
    cy_intrinsic = intrinsics[1, 2]

    # Convert from subpixel to pixel disparity
    subpixel_scale = 2 ** subpixel_bits
    valid_disp_px = roi[valid_mask] / subpixel_scale

    # depth = (focal_length * baseline) / disparity
    valid_depth_mm = (fx * baseline_mm) / valid_disp_px

    # Compute 3D position using circle center and median depth
    Z_mm = float(np.median(valid_depth_mm))
    X_mm = (cx - cx_intrinsic) * Z_mm / fx
    Y_mm = (cy - cy_intrinsic) * Z_mm / fy

    # Convert to meters
    return {
        'x': X_mm / 1000.0,
        'y': Y_mm / 1000.0,
        'z': Z_mm / 1000.0,
    }


def main():
    # Default paths
    default_model = Path(__file__).parent.parent.parent.parent / "weights" / "yolo26m_ball_background.pt"
    default_cam2bot = Path(__file__).parent.parent / "cam2bot.yaml"

    parser = argparse.ArgumentParser(description="Ball detection with 3D position in robot frame")
    parser.add_argument("--model", "-m", type=str, default=str(default_model),
                        help="Path to YOLO model weights")
    parser.add_argument("--cam2bot", type=str, default=str(default_cam2bot),
                        help="Path to cam2bot transformation yaml")
    parser.add_argument("--width", type=int, default=640,
                        help="Camera width (default: 640)")
    parser.add_argument("--height", type=int, default=400,
                        help="Camera height (default: 400)")
    parser.add_argument("--fps", type=int, default=60,
                        help="Camera FPS (default: 60)")
    parser.add_argument("--conf", type=float, default=0.5,
                        help="Confidence threshold (default: 0.5)")

    args = parser.parse_args()

    # Load cam2bot transformation
    load_cam2bot_transform(Path(args.cam2bot))

    # Load YOLO model
    model_path = Path(args.model)
    if not model_path.exists():
        print(f"Error: Model not found at {model_path}")
        return

    print(f"Loading YOLO model: {model_path}")
    model = YOLO(str(model_path))
    print("Model loaded successfully")

    # Setup OAK-D pipeline with stereo depth
    with dai.Pipeline() as pipeline:
        # Create mono cameras (CAM_B = left, CAM_C = right)
        monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

        # Request output at desired resolution and FPS
        monoLeftOut = monoLeft.requestOutput((args.width, args.height), fps=args.fps)
        monoRightOut = monoRight.requestOutput((args.width, args.height), fps=args.fps)

        # Create stereo depth node
        stereo = pipeline.create(dai.node.StereoDepth)

        # Link cameras to stereo
        monoLeftOut.link(stereo.left)
        monoRightOut.link(stereo.right)

        # Stereo settings
        stereo.setRectification(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)  # Align depth to left camera
        stereo.initialConfig.censusTransform.kernelSize = dai.StereoDepthConfig.CensusTransform.KernelSize.KERNEL_5x5
        stereo.initialConfig.censusTransform.enableMeanMode = True
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        stereo.initialConfig.setConfidenceThreshold(10)
        stereo.initialConfig.costMatching.disparityWidth = dai.StereoDepthConfig.CostMatching.DisparityWidth.DISPARITY_64

        # Output queues
        dispQ = stereo.disparity.createOutputQueue(maxSize=1, blocking=False)
        monoQ = stereo.rectifiedLeft.createOutputQueue(maxSize=1, blocking=False)

        # Get max disparity for normalization
        max_disparity = stereo.initialConfig.getMaxDisparity()
        disp_scale = 255.0 / max_disparity

        pipeline.start()

        # Get calibration data
        device = pipeline.getDefaultDevice()
        calib = device.readCalibration()
        intrinsics = np.array(calib.getCameraIntrinsics(
            dai.CameraBoardSocket.CAM_B,
            args.width,
            args.height
        ))
        extrinsics = np.array(calib.getCameraExtrinsics(
            dai.CameraBoardSocket.CAM_B,
            dai.CameraBoardSocket.CAM_C
        ))
        baseline_mm = np.linalg.norm(extrinsics[:3, 3]) * 10  # cm to mm

        print(f"Connected to: {device.getDeviceName()}")
        print(f"Camera: {args.width}x{args.height} @ {args.fps} fps")
        print(f"Calibration: baseline={baseline_mm:.1f}mm, fx={intrinsics[0,0]:.1f}px")
        print()
        print("Controls:")
        print("  'r' - Start/stop recording")
        print("  'd' - Toggle depth display")
        print("  't' - Toggle trajectory display")
        print("  'c' - Clear trajectory")
        print("  'q' - Quit")
        print()

        # State
        fps = 0
        prev_time = None
        show_depth = False
        show_trajectory = True

        # Kalman filter and trajectory
        # High process_noise = more responsive, less smoothing
        # Low measurement_noise = trust measurements more
        tracker = BallTracker3D(dt=1/args.fps, process_noise=50.0, measurement_noise=0.005)
        trajectory = deque(maxlen=16)  # Last 16 frames

        # Recording
        writer = None
        recording = False
        frame_count = 0
        output_dir = Path(__file__).parent.parent / "recordings"
        output_dir.mkdir(exist_ok=True)

        cv2.namedWindow("Ball 3D Detection")

        while pipeline.isRunning():
            dispFrame = dispQ.get()
            monoFrame = monoQ.get()

            if dispFrame is None or monoFrame is None:
                continue

            # Calculate FPS
            current_time = cv2.getTickCount()
            if prev_time is not None:
                dt = (current_time - prev_time) / cv2.getTickFrequency()
                if dt > 0:
                    fps = 0.1 * fps + 0.9 * (1.0 / dt)
            prev_time = current_time

            # Calculate latency
            timestamp = monoFrame.getTimestamp()
            cam_latency_ms = (dai.Clock.now() - timestamp).total_seconds() * 1000

            # Get frames
            frame_gray = monoFrame.getCvFrame()
            disparity = dispFrame.getFrame()

            # Display frame
            if show_depth:
                frame_bgr = cv2.applyColorMap(
                    (disparity * disp_scale).astype(np.uint8),
                    cv2.COLORMAP_JET
                )
            else:
                frame_bgr = cv2.cvtColor(frame_gray, cv2.COLOR_GRAY2BGR)

            # Run YOLO inference
            infer_start = cv2.getTickCount()
            results = model(frame_bgr, conf=args.conf, verbose=False)
            infer_time = (cv2.getTickCount() - infer_start) / cv2.getTickFrequency() * 1000

            # Process detections
            detection_found = False
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # Get bounding box
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    conf = box.conf[0].cpu().numpy()

                    # Detect circle within bbox
                    circle = detect_circle_in_bbox(frame_gray, (x1, y1, x2, y2))

                    if circle:
                        cx, cy, radius = circle

                        # Get 3D position in camera frame
                        p_cam_raw = get_depth_at_circle(
                            disparity, cx, cy, radius,
                            intrinsics, baseline_mm
                        )

                        if p_cam_raw:
                            detection_found = True
                            p_cam = [p_cam_raw['x'], p_cam_raw['y'], p_cam_raw['z']]

                            # Add RAW position to trajectory (no filtering for fast ball)
                            trajectory.append(p_cam)

                            # Update Kalman filter just for velocity estimation
                            tracker.update(p_cam)
                            velocity = tracker.get_velocity()

                            # Transform to robot frame
                            p_robot = apply_cam2bot_transform(p_cam)

                            # Draw circle
                            cv2.circle(frame_bgr, (cx, cy), radius, (0, 255, 0), 2)
                            cv2.circle(frame_bgr, (cx, cy), 3, (0, 0, 255), -1)

                            # Display p_cam
                            label_cam = f"cam: ({p_cam[0]:.3f}, {p_cam[1]:.3f}, {p_cam[2]:.3f})"
                            cv2.putText(frame_bgr, label_cam, (cx - 80, cy - radius - 40),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

                            # Display velocity
                            if velocity is not None:
                                speed = np.linalg.norm(velocity)
                                label_vel = f"vel: {speed:.2f} m/s"
                                cv2.putText(frame_bgr, label_vel, (cx - 80, cy - radius - 25),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

                            # Display p_robot if transform available
                            if p_robot:
                                label_robot = f"robot: ({p_robot[0]:.3f}, {p_robot[1]:.3f}, {p_robot[2]:.3f})"
                                cv2.putText(frame_bgr, label_robot, (cx - 80, cy - radius - 10),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                            else:
                                cv2.putText(frame_bgr, "robot: N/A (no transform)", (cx - 80, cy - radius - 10),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)

            # No prediction when detection lost - trajectory just shows real detections
            if not detection_found:
                tracker.predict()  # Keep Kalman state updated

            # Draw trajectory (with 3-point moving average)
            if show_trajectory and len(trajectory) > 1:
                smoothed_traj = moving_average(list(trajectory), window=3)
                draw_trajectory(frame_bgr, smoothed_traj, intrinsics)

            # Draw info overlay
            cv2.putText(frame_bgr, f"FPS: {fps:.1f}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame_bgr, f"Latency: {cam_latency_ms:.1f}ms", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame_bgr, f"Infer: {infer_time:.1f}ms", (10, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            mode_text = "DEPTH" if show_depth else "MONO"
            cv2.putText(frame_bgr, mode_text, (args.width - 70, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # Trajectory status
            if show_trajectory:
                traj_text = f"TRAJ: {len(trajectory)}"
                cv2.putText(frame_bgr, traj_text, (args.width - 90, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

            # Recording indicator
            if recording:
                cv2.putText(frame_bgr, f"REC ({frame_count})", (args.width - 120, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.circle(frame_bgr, (args.width - 130, 45), 8, (0, 0, 255), -1)
                writer.write(frame_bgr)
                frame_count += 1

            cv2.imshow("Ball 3D Detection", frame_bgr)

            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            elif key == ord('d'):
                show_depth = not show_depth
                print(f"Depth display: {'ON' if show_depth else 'OFF'}")
            elif key == ord('t'):
                show_trajectory = not show_trajectory
                print(f"Trajectory display: {'ON' if show_trajectory else 'OFF'}")
            elif key == ord('c'):
                trajectory.clear()
                tracker.reset()
                print("Trajectory cleared")
            elif key == ord('r'):
                if not recording:
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    output_path = output_dir / f"ball_3d_{timestamp}.mp4"
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    writer = cv2.VideoWriter(
                        str(output_path), fourcc, args.fps, (args.width, args.height)
                    )
                    recording = True
                    frame_count = 0
                    print(f"Recording started: {output_path}")
                else:
                    recording = False
                    if writer is not None:
                        writer.release()
                        writer = None
                    print(f"Recording stopped. Saved {frame_count} frames.")

        if writer is not None:
            writer.release()

    cv2.destroyAllWindows()
    print("Done.")


if __name__ == "__main__":
    main()

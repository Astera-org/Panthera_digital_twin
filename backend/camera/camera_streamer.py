#!/usr/bin/env python3
"""
Camera streamer module for OAK-D stereo cameras.
Streams mono image or colorized depth frames for WebSocket streaming.
Supports depth measurement via region selection.
"""

import threading
import time
import numpy as np
import cv2

try:
    import depthai as dai
    DEPTHAI_AVAILABLE = True
except ImportError:
    DEPTHAI_AVAILABLE = False
    print("Warning: depthai not installed. Camera streaming will be in demo mode.")


class CameraStreamer:
    """Streams mono image or depth from OAK-D stereo camera."""

    def __init__(self, resolution=(640, 400), fps=60, downsample=1):
        """
        Initialize camera streamer.

        Args:
            resolution: (width, height) for camera output
            fps: Target frame rate
            downsample: Kept for compatibility (not used for image streaming)
        """
        self.resolution = resolution
        self.fps = fps
        self.downsample = downsample

        self.running = False
        self.pipeline = None
        self.device = None
        self.thread = None

        # Mode: 'mono' or 'depth'
        self.mode = 'mono'
        self.mode_lock = threading.Lock()

        # Latest data
        self.latest_mono = None
        self.latest_depth = None
        self.latest_disparity = None  # Raw disparity for depth calculation
        self.lock = threading.Lock()

        # Calibration data for depth calculation
        self.intrinsics = None
        self.baseline_mm = None
        self.subpixel_bits = 5  # Default for OAK-D

        # Callbacks
        self.on_frame = None

        # Stats
        self.frame_count = 0
        self.actual_fps = 0
        self.latency_ms = 0

        # Depth colormap scale
        self.disp_scale = 1.0
        self.max_disparity = 1

    def is_available(self):
        """Check if camera hardware is available."""
        if not DEPTHAI_AVAILABLE:
            return False
        try:
            devices = dai.Device.getAllAvailableDevices()
            return len(devices) > 0
        except Exception:
            return False

    def set_mode(self, mode):
        """Set streaming mode ('mono' or 'depth')."""
        if mode not in ('mono', 'depth'):
            return False
        with self.mode_lock:
            self.mode = mode
        return True

    def get_mode(self):
        """Get current streaming mode."""
        with self.mode_lock:
            return self.mode

    def start(self):
        """Start camera streaming in background thread."""
        if self.running:
            return True

        if not DEPTHAI_AVAILABLE:
            print("Starting camera in demo mode (no depthai)")
            self.running = True
            self._setup_demo_calibration()
            self.thread = threading.Thread(target=self._demo_loop, daemon=True)
            self.thread.start()
            return True

        try:
            self.running = True
            self.thread = threading.Thread(target=self._stream_loop, daemon=True)
            self.thread.start()
            return True
        except Exception as e:
            print(f"Failed to start camera: {e}")
            self.running = False
            return False

    def stop(self):
        """Stop camera streaming."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
            self.thread = None

    def get_latest_data(self):
        """Get the latest image based on current mode."""
        with self.lock:
            mode = self.get_mode()
            if mode == 'depth':
                image = self.latest_depth
            else:
                image = self.latest_mono
            return {
                'image': image,
                'mode': mode,
                'fps': self.actual_fps,
                'latency_ms': self.latency_ms,
                'frame_count': self.frame_count
            }

    def get_depth_at_region(self, box):
        """
        Calculate depth statistics and 3D position for a box region.

        Args:
            box: (x1, y1, x2, y2) region of interest in image coordinates

        Returns:
            Dict with XYZ in camera frame (meters) and stats, or None if invalid
        """
        with self.lock:
            disparity = self.latest_disparity
            intrinsics = self.intrinsics
            baseline_mm = self.baseline_mm

        if disparity is None or intrinsics is None or baseline_mm is None:
            return None

        if box is None:
            return None

        x1, y1, x2, y2 = box
        # Clamp to image bounds
        h, w = disparity.shape[:2]
        x1, y1 = max(0, int(x1)), max(0, int(y1))
        x2, y2 = min(w, int(x2)), min(h, int(y2))

        if x2 <= x1 or y2 <= y1:
            return None

        roi = disparity[y1:y2, x1:x2].astype(np.float32)
        valid_mask = roi > 0  # Filter out invalid disparities

        if not np.any(valid_mask):
            return None

        # Extract intrinsic parameters
        fx = intrinsics[0, 0]
        fy = intrinsics[1, 1]
        cx = intrinsics[0, 2]
        cy = intrinsics[1, 2]

        # Convert from subpixel to pixel disparity
        subpixel_scale = 2 ** self.subpixel_bits  # 32 for 5 bits
        valid_disp_px = roi[valid_mask] / subpixel_scale

        # depth = (focal_length * baseline) / disparity
        valid_depth_mm = (fx * baseline_mm) / valid_disp_px

        # Compute 3D position for center of box using median depth
        center_u = (x1 + x2) / 2.0
        center_v = (y1 + y2) / 2.0
        Z_mm = float(np.median(valid_depth_mm))
        X_mm = (center_u - cx) * Z_mm / fx
        Y_mm = (center_v - cy) * Z_mm / fy

        # Convert to meters
        return {
            'x': X_mm / 1000.0,
            'y': Y_mm / 1000.0,
            'z': Z_mm / 1000.0,
            'min_depth': float(np.min(valid_depth_mm)) / 1000.0,
            'max_depth': float(np.max(valid_depth_mm)) / 1000.0,
            'median_depth': Z_mm / 1000.0,
            'box': [x1, y1, x2, y2]
        }

    def _setup_demo_calibration(self):
        """Setup fake calibration for demo mode."""
        # Typical OAK-D Lite values for 640x400
        self.intrinsics = np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 200.0],
            [0.0, 0.0, 1.0]
        ])
        self.baseline_mm = 75.0  # 7.5cm baseline
        self.max_disparity = 96 * 32  # 96 pixels * 32 subpixel levels
        self.disp_scale = 255.0 / self.max_disparity

    def _stream_loop(self):
        """Main streaming loop with stereo depth pipeline."""
        with dai.Pipeline() as pipeline:
            # Create mono cameras (CAM_B = left, CAM_C = right)
            monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
            monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

            # Request output at desired resolution and FPS
            monoLeftOut = monoLeft.requestOutput(self.resolution, fps=self.fps)
            monoRightOut = monoRight.requestOutput(self.resolution, fps=self.fps)

            # Create stereo depth node
            stereo = pipeline.create(dai.node.StereoDepth)

            # Link cameras to stereo
            monoLeftOut.link(stereo.left)
            monoRightOut.link(stereo.right)

            # Stereo settings - balanced quality/speed
            stereo.setRectification(True)
            stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)  # Align depth to left camera (matches intrinsics)
            stereo.initialConfig.censusTransform.kernelSize = dai.StereoDepthConfig.CensusTransform.KernelSize.KERNEL_5x5
            stereo.initialConfig.censusTransform.enableMeanMode = True
            stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
            stereo.initialConfig.setConfidenceThreshold(10)
            stereo.initialConfig.costMatching.disparityWidth = dai.StereoDepthConfig.CostMatching.DisparityWidth.DISPARITY_64

            # Output queues
            dispQ = stereo.disparity.createOutputQueue(maxSize=4, blocking=False)
            monoQ = stereo.rectifiedLeft.createOutputQueue(maxSize=4, blocking=False)

            # Get max disparity for normalization
            self.max_disparity = stereo.initialConfig.getMaxDisparity()
            self.disp_scale = 255.0 / self.max_disparity

            pipeline.start()

            # Get calibration data for depth calculation
            device = pipeline.getDefaultDevice()
            calib = device.readCalibration()
            self.intrinsics = np.array(calib.getCameraIntrinsics(
                dai.CameraBoardSocket.CAM_B,
                self.resolution[0],
                self.resolution[1]
            ))
            extrinsics = np.array(calib.getCameraExtrinsics(
                dai.CameraBoardSocket.CAM_B,
                dai.CameraBoardSocket.CAM_C
            ))
            self.baseline_mm = np.linalg.norm(extrinsics[:3, 3]) * 10  # cm to mm

            print(f"Camera started (stereo): {self.resolution[0]}x{self.resolution[1]} @ {self.fps}fps")
            print(f"Calibration: baseline={self.baseline_mm:.1f}mm, fx={self.intrinsics[0,0]:.1f}px")

            prev_time = time.time()
            fps_alpha = 0.1

            while self.running and pipeline.isRunning():
                dispFrame = dispQ.get()
                monoFrame = monoQ.get()

                if dispFrame is None or monoFrame is None:
                    continue

                # Calculate latency
                timestamp = dispFrame.getTimestamp()
                self.latency_ms = (dai.Clock.now() - timestamp).total_seconds() * 1000

                # Calculate FPS
                now = time.time()
                dt = now - prev_time
                if dt > 0:
                    self.actual_fps = fps_alpha * self.actual_fps + (1 - fps_alpha) * (1.0 / dt)
                prev_time = now

                # Get mono frame
                mono = monoFrame.getCvFrame()

                # Get disparity (raw for depth calculation)
                disparity = dispFrame.getFrame()

                # Colorize for display
                depth_color = cv2.applyColorMap(
                    (disparity * self.disp_scale).astype(np.uint8),
                    cv2.COLORMAP_JET
                )

                with self.lock:
                    self.latest_mono = mono
                    self.latest_depth = depth_color
                    self.latest_disparity = disparity.copy()
                    self.frame_count += 1

                # Callback
                if self.on_frame:
                    mode = self.get_mode()
                    if mode == 'depth':
                        self.on_frame(depth_color)
                    else:
                        self.on_frame(mono)

    def _demo_loop(self):
        """Demo loop generating synthetic mono and depth images."""
        print("Camera running in demo mode")

        while self.running:
            start_time = time.time()

            # Generate synthetic mono image (animated gradient)
            t = time.time()
            mono = np.zeros((self.resolution[1], self.resolution[0]), dtype=np.uint8)
            for i in range(self.resolution[1]):
                mono[i, :] = int(128 + 64 * np.sin(t + i * 0.05))

            # Generate synthetic disparity (wave pattern with subpixel values)
            disparity = np.zeros((self.resolution[1], self.resolution[0]), dtype=np.uint16)
            for i in range(self.resolution[1]):
                for j in range(self.resolution[0]):
                    # Simulate varying depth (higher disparity = closer)
                    disp_val = int(1000 + 500 * np.sin(t * 0.5 + i * 0.01 + j * 0.01))
                    disparity[i, j] = max(100, disp_val)

            # Colorize for display
            depth_color = cv2.applyColorMap(
                (disparity * self.disp_scale).astype(np.uint8),
                cv2.COLORMAP_JET
            )

            with self.lock:
                self.latest_mono = mono
                self.latest_depth = depth_color
                self.latest_disparity = disparity
                self.frame_count += 1
                self.actual_fps = self.fps
                self.latency_ms = 1.0

            if self.on_frame:
                mode = self.get_mode()
                if mode == 'depth':
                    self.on_frame(depth_color)
                else:
                    self.on_frame(mono)

            # Sleep to maintain target FPS
            elapsed = time.time() - start_time
            sleep_time = 1.0 / self.fps - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def get_config(self):
        """Get camera configuration for frontend."""
        return {
            'resolution': list(self.resolution),
            'fps': self.fps,
            'mode': self.get_mode(),
            'available': self.is_available() or not DEPTHAI_AVAILABLE,
            'demo_mode': not DEPTHAI_AVAILABLE,
            'has_calibration': self.intrinsics is not None
        }

#!/usr/bin/env python3
"""
Fast on-device stereo depth based on working stereo_test.py example.
Press 't' to toggle box drawing mode, then drag to select a region.
"""

import cv2
import depthai as dai
import numpy as np

# Global variables for box drawing
drawing_mode = False  # Toggle with 't' key
drawing = False       # True while mouse button is held
start_point = None
end_point = None
current_box = None    # (x1, y1, x2, y2) of selected box


def mouse_callback(event, x, y, flags, param):
    global drawing, start_point, end_point, current_box

    if not drawing_mode:
        return

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        start_point = (x, y)
        end_point = (x, y)

    elif event == cv2.EVENT_MOUSEMOVE and drawing:
        end_point = (x, y)

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        end_point = (x, y)
        # Store the box coordinates (ensure proper ordering)
        x1, y1 = start_point
        x2, y2 = end_point
        current_box = (min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2))


def get_depth_stats(disparity, box, intrinsics, baseline_mm, subpixel_bits=5):
    """Calculate depth statistics and 3D position for a box region.

    Args:
        disparity: Disparity map (raw subpixel values)
        box: (x1, y1, x2, y2) region of interest
        intrinsics: 3x3 camera intrinsic matrix
        baseline_mm: Stereo baseline in mm
        subpixel_bits: Number of fractional bits (default 5 = 32 subpixel levels)

    Returns:
        Dict with depth stats in mm and XYZ in camera frame, or None if invalid
    """
    if box is None:
        return None

    x1, y1, x2, y2 = box
    # Clamp to image bounds
    h, w = disparity.shape[:2]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w, x2), min(h, y2)

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
    subpixel_scale = 2 ** subpixel_bits  # 32 for 5 bits
    valid_disp_px = roi[valid_mask] / subpixel_scale

    # depth = (focal_length * baseline) / disparity
    valid_depth_mm = (fx * baseline_mm) / valid_disp_px

    # Compute 3D position for center of box using median depth
    center_u = (x1 + x2) / 2.0
    center_v = (y1 + y2) / 2.0
    Z = float(np.median(valid_depth_mm))
    X = (center_u - cx) * Z / fx
    Y = (center_v - cy) * Z / fy

    return {
        'min_mm': float(np.min(valid_depth_mm)),
        'max_mm': float(np.max(valid_depth_mm)),
        'median_mm': Z,
        'median_disp': float(np.median(valid_disp_px)),
        'X_mm': X,
        'Y_mm': Y,
        'Z_mm': Z,
    }


def main():
    global drawing_mode, current_box
    with dai.Pipeline() as pipeline:
        # Create mono cameras (CAM_B = left, CAM_C = right)
        monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

        # Request output at desired resolution and FPS
        monoLeftOut = monoLeft.requestOutput((640, 400), fps=80)
        monoRightOut = monoRight.requestOutput((640, 400), fps=80)
        # monoLeftOut = monoLeft.requestOutput((400, 320), fps=90)
        # monoRightOut = monoRight.requestOutput((400, 320), fps=90)

        # Create stereo depth node
        stereo = pipeline.create(dai.node.StereoDepth)

        # Link cameras to stereo
        monoLeftOut.link(stereo.left)
        monoRightOut.link(stereo.right)

        # Stereo settings - balanced quality/speed
        stereo.setRectification(True)
        stereo.initialConfig.censusTransform.kernelSize = dai.StereoDepthConfig.CensusTransform.KernelSize.KERNEL_5x5
        stereo.initialConfig.censusTransform.enableMeanMode = True
        # stereo.setLeftRightCheck(True)  # Removes mismatches
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        stereo.initialConfig.setConfidenceThreshold(10)  # Lower = keep more (0-255)
        stereo.initialConfig.costMatching.disparityWidth = dai.StereoDepthConfig.CostMatching.DisparityWidth.DISPARITY_64

        # Post-processing
        # stereo.initialConfig.postProcessing.speckleFilter.enable = True
        # stereo.initialConfig.postProcessing.speckleFilter.speckleRange = 30
        # stereo.initialConfig.postProcessing.temporalFilter.enable = True

        # Output queues - maxSize=1, blocking=False for lowest latency
        dispQ = stereo.disparity.createOutputQueue(maxSize=1, blocking=False)
        monoQ = stereo.rectifiedLeft.createOutputQueue(maxSize=1, blocking=False)

        # Get max disparity for normalization
        maxDisparity = stereo.initialConfig.getMaxDisparity()
        print(f"Max disparity: {maxDisparity}")

        pipeline.start()

        # Get calibration data for depth calculation
        device = pipeline.getDefaultDevice()
        calib = device.readCalibration()
        intrinsics = np.array(calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, 640, 400))
        focal_length_px = intrinsics[0][0]  # fx from intrinsic matrix
        extrinsics = np.array(calib.getCameraExtrinsics(dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C))
        baseline_mm = np.linalg.norm(extrinsics[:3, 3]) * 10  # cm to mm
        print(f"Baseline: {baseline_mm:.1f}mm, Focal length: {focal_length_px:.1f}px")

        # Pre-compute constants for speed
        cx_int, cy_int = int(intrinsics[0, 2]), int(intrinsics[1, 2])
        disp_scale = 255.0 / maxDisparity

        # Set up window and mouse callback
        cv2.namedWindow("On-Device Stereo")
        cv2.setMouseCallback("On-Device Stereo", mouse_callback)

        fps = 0
        prev_time = None

        while pipeline.isRunning():
            loop_start = cv2.getTickCount()

            dispFrame = dispQ.get()
            monoFrame = monoQ.get()

            if dispFrame is None:
                continue

            # Calculate latency (device to host)
            timestamp = dispFrame.getTimestamp()
            latency = (dai.Clock.now() - timestamp).total_seconds() * 1000

            # Calculate FPS from loop time
            if prev_time is not None:
                dt = (loop_start - prev_time) / cv2.getTickFrequency()
                if dt > 0:
                    fps = 0.1 * fps + 0.9 * (1.0 / dt)
            prev_time = loop_start

            # Get and colorize disparity
            disparity = dispFrame.getFrame()
            dispColor = cv2.applyColorMap(
                (disparity * disp_scale).astype(np.uint8),
                cv2.COLORMAP_JET
            )

            # Get mono frame
            if monoFrame is not None:
                monoImg = cv2.cvtColor(monoFrame.getCvFrame(), cv2.COLOR_GRAY2BGR)

                # Resize if needed
                if dispColor.shape[:2] != monoImg.shape[:2]:
                    dispColor = cv2.resize(dispColor, (monoImg.shape[1], monoImg.shape[0]))

                # Draw optical center crosshair on mono image
                cross_size = 10
                cv2.line(monoImg, (cx_int - cross_size, cy_int), (cx_int + cross_size, cy_int), (0, 255, 255), 1)
                cv2.line(monoImg, (cx_int, cy_int - cross_size), (cx_int, cy_int + cross_size), (0, 255, 255), 1)
                cv2.putText(monoImg, f"cx={cx_int} cy={cy_int}", (cx_int + 5, cy_int - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

                cv2.putText(monoImg, f"FPS:{fps:.0f} Lat:{latency:.0f}ms",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                display = np.hstack([monoImg, dispColor])
            else:
                cv2.putText(dispColor, f"FPS:{fps:.0f} Lat:{latency:.0f}ms",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                display = dispColor

            # Show drawing mode status
            mode_text = "BOX MODE (t to exit)" if drawing_mode else "Press 't' for box mode"
            color = (0, 0, 255) if drawing_mode else (200, 200, 200)
            cv2.putText(display, mode_text, (10, display.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

            # Draw current selection box while dragging (on both sides)
            if drawing_mode and drawing and start_point and end_point:
                cv2.rectangle(display, start_point, end_point, (0, 255, 0), 2)
                # Mirror to right side if drawing on left
                if monoFrame is not None and start_point[0] < monoImg.shape[1]:
                    offset = monoImg.shape[1]
                    cv2.rectangle(display,
                                  (start_point[0] + offset, start_point[1]),
                                  (end_point[0] + offset, end_point[1]),
                                  (0, 255, 0), 2)

            # Draw and analyze completed box
            if current_box is not None:
                x1, y1, x2, y2 = current_box
                mono_width = monoImg.shape[1] if monoFrame is not None else 0

                # Determine if box is on left or right side
                if x1 < mono_width:
                    # Box drawn on left - use same coords for disparity
                    left_box = (x1, y1, x2, y2)
                    disp_box = (x1, y1, x2, y2)  # Same region in disparity
                    right_box = (x1 + mono_width, y1, x2 + mono_width, y2)
                else:
                    # Box drawn on right - mirror to left
                    disp_box = (x1 - mono_width, y1, x2 - mono_width, y2)
                    left_box = disp_box
                    right_box = (x1, y1, x2, y2)

                # Draw box on both sides
                cv2.rectangle(display, (left_box[0], left_box[1]), (left_box[2], left_box[3]), (255, 255, 255), 2)
                cv2.rectangle(display, (right_box[0], right_box[1]), (right_box[2], right_box[3]), (255, 255, 255), 2)

                # Calculate depth stats from disparity
                stats = get_depth_stats(disparity, disp_box, intrinsics, baseline_mm)
                if stats:
                    X_m = stats['X_mm'] / 1000.0
                    Y_m = stats['Y_mm'] / 1000.0
                    Z_m = stats['Z_mm'] / 1000.0
                    # Show XYZ in three rows
                    text_x = left_box[0]
                    text_y = left_box[1] - 50
                    cv2.putText(display, f"X={X_m:.3f}", (text_x, text_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    cv2.putText(display, f"Y={Y_m:.3f}", (text_x, text_y + 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    cv2.putText(display, f"Z={Z_m:.3f}", (text_x, text_y + 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            cv2.imshow("On-Device Stereo", display)

            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            elif key == ord('t'):
                drawing_mode = not drawing_mode
                if not drawing_mode:
                    current_box = None  # Clear box when exiting mode
                print(f"Box drawing mode: {'ON' if drawing_mode else 'OFF'}")


if __name__ == "__main__":
    main()

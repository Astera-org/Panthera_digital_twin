#!/usr/bin/env python3
"""
Real-time ball detection using YOLO on OAK-D camera (CAM_B = left camera).

Uses stereo node for rectification to match training data.
Press 'r' to start/stop recording, 'q' to quit.

Usage:
    python yolo_ball_detect.py
    python yolo_ball_detect.py --model path/to/model.pt --width 640 --height 400 --fps 60
"""

import argparse
from datetime import datetime
from pathlib import Path

import cv2
import depthai as dai
from ultralytics import YOLO


def main():
    # Default model path relative to project root (recordings -> camera -> backend -> project root)
    default_model = Path(__file__).parent.parent.parent.parent / "weights" / "yolo26m_ball_background.pt"

    parser = argparse.ArgumentParser(description="YOLO ball detection on OAK-D camera")
    parser.add_argument("--model", "-m", type=str,
                        default=str(default_model),
                        help="Path to YOLO model weights")
    parser.add_argument("--width", type=int, default=640,
                        help="Camera width (default: 640)")
    parser.add_argument("--height", type=int, default=400,
                        help="Camera height (default: 400)")
    parser.add_argument("--fps", type=int, default=60,
                        help="Camera FPS (default: 60)")
    parser.add_argument("--conf", type=float, default=0.5,
                        help="Confidence threshold (default: 0.5)")

    args = parser.parse_args()

    # Load YOLO model
    model_path = Path(args.model)
    if not model_path.exists():
        print(f"Error: Model not found at {model_path}")
        return

    print(f"Loading YOLO model: {model_path}")
    model = YOLO(str(model_path))
    print("Model loaded successfully")

    # Setup OAK-D pipeline
    with dai.Pipeline() as pipeline:
        # Create mono cameras (CAM_B = left, CAM_C = right)
        monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

        # Request output at desired resolution and FPS
        monoLeftOut = monoLeft.requestOutput((args.width, args.height), fps=args.fps)
        monoRightOut = monoRight.requestOutput((args.width, args.height), fps=args.fps)

        # Create stereo depth node (for rectification only)
        stereo = pipeline.create(dai.node.StereoDepth)

        # Link cameras to stereo
        monoLeftOut.link(stereo.left)
        monoRightOut.link(stereo.right)

        # Stereo settings - minimal for rectification
        stereo.setRectification(True)

        # Output queue for rectified left - maxSize=1, blocking=False for lowest latency
        monoQ = stereo.rectifiedLeft.createOutputQueue(maxSize=1, blocking=False)

        pipeline.start()

        # Get device info
        device = pipeline.getDefaultDevice()
        print(f"Connected to: {device.getDeviceName()}")
        print(f"Camera: {args.width}x{args.height} @ {args.fps} fps")
        print(f"Confidence threshold: {args.conf}")
        print()
        print("Controls:")
        print("  'r' - Start/stop recording")
        print("  'q' - Quit")
        print()

        # FPS and timing
        fps = 0
        prev_time = None
        detection_count = 0

        # Video recording
        writer = None
        recording = False
        frame_count = 0
        output_dir = Path(__file__).parent  # Save to same folder as script

        cv2.namedWindow("YOLO Ball Detection")

        while pipeline.isRunning():
            monoFrame = monoQ.get()
            if monoFrame is None:
                continue

            # Calculate FPS (frame-to-frame time including processing)
            current_time = cv2.getTickCount()
            if prev_time is not None:
                dt = (current_time - prev_time) / cv2.getTickFrequency()
                if dt > 0:
                    fps = 0.1 * fps + 0.9 * (1.0 / dt)
            prev_time = current_time

            # Calculate camera latency
            timestamp = monoFrame.getTimestamp()
            cam_latency_ms = (dai.Clock.now() - timestamp).total_seconds() * 1000

            # Get frame
            frame = monoFrame.getCvFrame()

            # Convert grayscale to BGR for YOLO and display
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

            # Run YOLO inference
            infer_start = cv2.getTickCount()
            results = model(frame_bgr, conf=args.conf, verbose=False)
            infer_time = (cv2.getTickCount() - infer_start) / cv2.getTickFrequency() * 1000

            # Draw detections
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    # Get box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    conf = box.conf[0].cpu().numpy()
                    cls = int(box.cls[0].cpu().numpy())

                    # Draw box
                    cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # Draw label
                    label = f"{model.names[cls]} {conf:.2f}"
                    cv2.putText(frame_bgr, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    detection_count += 1

            # Draw info overlay
            cv2.putText(frame_bgr, f"FPS: {fps:.1f}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame_bgr, f"Cam latency: {cam_latency_ms:.1f}ms", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame_bgr, f"Infer time: {infer_time:.1f}ms", (10, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Recording indicator
            if recording:
                cv2.putText(frame_bgr, f"REC ({frame_count})", (args.width - 120, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                cv2.circle(frame_bgr, (args.width - 130, 20), 8, (0, 0, 255), -1)
                # Write frame with detections
                writer.write(frame_bgr)
                frame_count += 1

            cv2.imshow("YOLO Ball Detection", frame_bgr)

            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            elif key == ord('r'):
                if not recording:
                    # Start recording
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    output_path = output_dir / f"yolo_detect_{timestamp}.mp4"
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    writer = cv2.VideoWriter(
                        str(output_path),
                        fourcc,
                        args.fps,
                        (args.width, args.height)
                    )
                    recording = True
                    frame_count = 0
                    print(f"Recording started: {output_path}")
                else:
                    # Stop recording
                    recording = False
                    if writer is not None:
                        writer.release()
                        writer = None
                    print(f"Recording stopped. Saved {frame_count} frames.")

        # Cleanup recording if still active
        if writer is not None:
            writer.release()
            print(f"Recording saved: {frame_count} frames")

    cv2.destroyAllWindows()
    print(f"Done. Total detections: {detection_count}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Record rectified mono video from OAK-D camera (CAM_B = left camera).

Uses stereo node for rectification to match what ball detection will use.
Press 'r' to start/stop recording, 'q' to quit.

Usage:
    python record_oakd_mono.py
    python record_oakd_mono.py --output my_video.mp4 --width 640 --height 400 --fps 80
"""

import argparse
from datetime import datetime
from pathlib import Path

import cv2
import depthai as dai


def main():
    parser = argparse.ArgumentParser(description="Record OAK-D rectified mono video")
    parser.add_argument("--output", "-o", type=str, default=None,
                        help="Output video path (default: auto-generated timestamp)")
    parser.add_argument("--width", type=int, default=640,
                        help="Camera width (default: 640)")
    parser.add_argument("--height", type=int, default=400,
                        help="Camera height (default: 400)")
    parser.add_argument("--fps", type=int, default=60,
                        help="Camera and output video FPS (default: 60)")

    args = parser.parse_args()

    # Generate output filename if not specified
    if args.output is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = Path(__file__).parent  # Save to same folder as script
        output_path = output_dir / f"oakd_mono_{timestamp}.mp4"
    else:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"Output will be saved to: {output_path}")

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
        print()
        print("Controls:")
        print("  'r' - Start/stop recording")
        print("  'q' - Quit")
        print()

        # Video writer (initialized when recording starts)
        writer = None
        recording = False
        frame_count = 0

        # FPS calculation
        fps = 0
        prev_time = None

        cv2.namedWindow("OAK-D Mono Recording")

        while pipeline.isRunning():
            monoFrame = monoQ.get()
            if monoFrame is None:
                continue

            # Calculate FPS (frame-to-frame time including processing)
            current_time = cv2.getTickCount()
            if prev_time is not None:
                dt = (current_time - prev_time) / cv2.getTickFrequency()
                if dt > 0:
                    fps = 0.9 * fps + 0.1 * (1.0 / dt)
            prev_time = current_time

            # Calculate latency
            timestamp = monoFrame.getTimestamp()
            latency_ms = (dai.Clock.now() - timestamp).total_seconds() * 1000

            # Get frame
            frame = monoFrame.getCvFrame()

            # Record if active
            if recording and writer is not None:
                # Convert grayscale to BGR for video writer
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                writer.write(frame_bgr)
                frame_count += 1

            # Display
            display = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

            # Draw info
            cv2.putText(display, f"FPS: {fps:.0f}  Latency: {latency_ms:.0f}ms",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            if recording:
                cv2.putText(display, f"RECORDING ({frame_count} frames)",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                # Red border when recording
                cv2.rectangle(display, (0, 0), (args.width - 1, args.height - 1), (0, 0, 255), 3)
            else:
                cv2.putText(display, "Press 'r' to record",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

            cv2.imshow("OAK-D Mono Recording", display)

            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            elif key == ord('r'):
                if not recording:
                    # Start recording
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
                    print(f"Recording stopped. Saved {frame_count} frames to {output_path}")

        # Cleanup
        if writer is not None:
            writer.release()
            print(f"Recording saved: {frame_count} frames to {output_path}")

    cv2.destroyAllWindows()
    print("Done.")


if __name__ == "__main__":
    main()

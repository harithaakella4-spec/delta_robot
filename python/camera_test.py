"""
Camera preview.

Opens the camera and shows what it sees. Nothing else.

    python camera_test.py
    python camera_test.py --device 1 --width 1920 --height 1080

KEYS
    ESC or q   quit
    s          save the current frame as snapshot_N.jpg

If OpenCV cannot open a CSI camera on a Pi, check it works first with:
    libcamera-hello --timeout 2000
If that works but this does not, the camera is only reachable through
picamera2 - see the note at the bottom of this file.
"""

import argparse
import time

import cv2


def main():
    ap = argparse.ArgumentParser(description="Camera preview")
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        raise SystemExit(
            f"could not open camera {args.device}.\n"
            "Try --device 1, or check the camera with libcamera-hello."
        )

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    # The camera may not honour the requested size, so report what it gave.
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"camera {args.device} open at {actual_w} x {actual_h}")
    print("ESC or q to quit, s to save a snapshot")

    frames = 0
    fps = 0.0
    t0 = time.time()
    saved = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            print("camera read failed")
            break

        frames += 1
        if frames % 10 == 0:
            now = time.time()
            fps = 10.0 / (now - t0)
            t0 = now

        cv2.putText(frame, f"{actual_w}x{actual_h}   {fps:.1f} fps",
                    (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (0, 255, 255), 2)

        cv2.imshow("camera", frame)

        k = cv2.waitKey(1) & 0xFF
        if k in (27, ord("q")):
            break
        if k == ord("s"):
            name = f"snapshot_{saved}.jpg"
            cv2.imwrite(name, frame)
            print(f"saved {name}")
            saved += 1

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()


# ---------------------------------------------------------------------------
# If OpenCV cannot see the CSI camera, use picamera2 instead. Same loop,
# different capture call:
#
#     from picamera2 import Picamera2
#     picam = Picamera2()
#     picam.configure(picam.create_preview_configuration(
#         main={"size": (1280, 720), "format": "RGB888"}))
#     picam.start()
#     while True:
#         frame = picam.capture_array()      # replaces cap.read()
#         cv2.imshow("camera", frame)
#         if (cv2.waitKey(1) & 0xFF) == 27:
#             break
#
# Everything downstream - calibration, detection - works on the frame
# either way, so only this one call changes.
# ---------------------------------------------------------------------------
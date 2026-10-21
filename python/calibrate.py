"""
Camera -> millimetre calibration.

Establishes the transform from pixel coordinates to real-world millimetres
on a flat plane, then lets you verify it by eye and by clicking.

    pip install opencv-python numpy

    python calibrate.py --rect 200 150      # calibrate, then verify
    python calibrate.py --verify            # reuse saved calibration

WHAT YOU NEED
    A sheet of paper taped down flat under the camera, with a rectangle
    marked on it. Measure the rectangle with a ruler - the printed size on
    a page is not reliable, and this number sets your entire scale.
    Bigger is better: a rectangle filling most of the frame gives a far
    more accurate homography than a small one in the middle.

HOW IT WORKS
    A homography maps one plane to another through a perspective view.
    Four point correspondences determine it completely. You click the four
    rectangle corners; the script already knows their millimetre positions
    from --rect, so it can solve for the mapping.

COORDINATE FRAME
    Origin is the first corner you click (top-left).
    X increases to the right, Y increases down the page, in millimetres.

    This is the PAPER frame, not the robot frame. When the robot is
    assembled, redo this by jogging the effector to four known points and
    clicking where it lands - that gives you camera -> robot directly and
    removes a whole layer of measurement error.

THE HEIGHT TRAP
    A homography maps ONE plane. Objects with height present their top
    surface to the camera, displaced radially outward from the image
    centre, so the detected centroid is not above where the object really
    sits. Zero directly under the lens, worse toward the edges.

    Mitigations: mount the camera high so rays are near vertical;
    calibrate at the height you will actually grip, by putting the markers
    on blocks of the same height as your objects; or detect the base
    contour rather than the whole blob.
"""

import argparse
import json
import os

import cv2
import numpy as np

CALIB_FILE = "calibration.json"

CORNER_NAMES = ["TOP-LEFT (origin)", "TOP-RIGHT", "BOTTOM-RIGHT", "BOTTOM-LEFT"]


# ==========================================================================
# Camera
# ==========================================================================

def open_camera(device, width, height):
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        raise SystemExit(
            f"could not open camera {device}.\n"
            "On a Pi with a CSI camera, check it appears via libcamera-hello "
            "first. If OpenCV cannot see it, use picamera2 to grab frames "
            "and pass them to the same functions in this file."
        )
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap


def grab_still(cap, window="camera"):
    """Live preview until SPACE freezes a frame. ESC aborts."""
    print("\nLive preview. SPACE to freeze the frame, ESC to quit.")
    while True:
        ok, frame = cap.read()
        if not ok:
            raise SystemExit("camera read failed")
        shown = frame.copy()
        cv2.putText(shown, "SPACE = freeze   ESC = quit", (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow(window, shown)
        k = cv2.waitKey(1) & 0xFF
        if k == 32:
            return frame
        if k == 27:
            raise SystemExit("aborted")


# ==========================================================================
# Calibration
# ==========================================================================

def collect_corners(frame, window="calibrate"):
    """Click four rectangle corners in order. Returns 4x2 pixel array."""
    pts = []

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(pts) < 4:
            pts.append((float(x), float(y)))
            print(f"  {CORNER_NAMES[len(pts)-1]:>18}  ->  pixel ({x}, {y})")

    cv2.namedWindow(window)
    cv2.setMouseCallback(window, on_mouse)

    print("\nClick the four rectangle corners IN ORDER.")
    print("Order matters - it defines the coordinate frame.")
    print("  BACKSPACE undoes the last point, ESC quits.\n")

    while True:
        shown = frame.copy()
        for i, p in enumerate(pts):
            cv2.circle(shown, (int(p[0]), int(p[1])), 6, (0, 0, 255), -1)
            cv2.putText(shown, str(i + 1),
                        (int(p[0]) + 10, int(p[1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        if len(pts) > 1:
            cv2.polylines(shown, [np.int32(pts)], len(pts) == 4,
                          (0, 255, 0), 2)

        msg = ("all four set - press ENTER" if len(pts) == 4
               else f"click {CORNER_NAMES[len(pts)]}")
        cv2.putText(shown, msg, (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 255), 2)
        cv2.imshow(window, shown)

        k = cv2.waitKey(20) & 0xFF
        if k == 27:
            raise SystemExit("aborted")
        if k == 8 and pts:
            pts.pop()
        if k in (13, 10) and len(pts) == 4:
            cv2.destroyWindow(window)
            return np.array(pts, dtype=np.float32)


def solve_homography(pixel_pts, width_mm, height_mm):
    """
    Map the four clicked corners onto their known millimetre positions.
    Returns (H, H_inverse, reprojection_errors_mm).
    """
    world = np.array([
        [0.0,      0.0],
        [width_mm, 0.0],
        [width_mm, height_mm],
        [0.0,      height_mm],
    ], dtype=np.float32)

    H, _ = cv2.findHomography(pixel_pts, world)
    if H is None:
        raise SystemExit("homography failed - are the four points distinct "
                         "and not collinear?")
    H_inv = np.linalg.inv(H)

    # With exactly four points the fit is exact, so this mostly catches
    # a mis-ordered click or a degenerate selection.
    got = px_to_mm(pixel_pts, H)
    errs = np.linalg.norm(got - world, axis=1)
    return H, H_inv, errs


def px_to_mm(pts, H):
    pts = np.asarray(pts, dtype=np.float32).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, H).reshape(-1, 2)


def mm_to_px(pts, H_inv):
    pts = np.asarray(pts, dtype=np.float32).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, H_inv).reshape(-1, 2)


def save_calibration(H, width_mm, height_mm):
    with open(CALIB_FILE, "w") as f:
        json.dump({"H": H.tolist(),
                   "width_mm": width_mm,
                   "height_mm": height_mm}, f, indent=2)
    print(f"\nsaved to {CALIB_FILE}")


def load_calibration():
    if not os.path.exists(CALIB_FILE):
        raise SystemExit(f"{CALIB_FILE} not found - run with --rect first")
    with open(CALIB_FILE) as f:
        d = json.load(f)
    H = np.array(d["H"], dtype=np.float64)
    return H, np.linalg.inv(H), d["width_mm"], d["height_mm"]


# ==========================================================================
# Verification
# ==========================================================================

def draw_mm_grid(img, H_inv, width_mm, height_mm, spacing=25):
    """
    Project a millimetre grid back into the image. This is the real test:
    if the drawn lines sit on your ruler markings, the calibration is
    correct. Numbers can look plausible while being wrong; a grid that
    does not line up is obvious.
    """
    def line_mm(p0, p1, colour, thick=1):
        a, b = mm_to_px([p0, p1], H_inv)
        cv2.line(img, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])),
                 colour, thick, cv2.LINE_AA)

    x = 0.0
    while x <= width_mm + 0.1:
        major = abs(x % 100) < 0.1
        line_mm((x, 0.0), (x, height_mm),
                (0, 180, 0) if major else (0, 90, 0), 2 if major else 1)
        x += spacing

    y = 0.0
    while y <= height_mm + 0.1:
        major = abs(y % 100) < 0.1
        line_mm((0.0, y), (width_mm, y),
                (0, 180, 0) if major else (0, 90, 0), 2 if major else 1)
        y += spacing

    for (mx, my) in [(0, 0), (width_mm, 0), (0, height_mm)]:
        p = mm_to_px([(mx, my)], H_inv)[0]
        cv2.putText(img, f"({mx:.0f},{my:.0f})",
                    (int(p[0]) + 6, int(p[1]) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)


def verify(cap, H, H_inv, width_mm, height_mm, spacing):
    """Live view with the grid drawn on, plus click-to-millimetres."""
    window = "verify"
    state = {"last": None}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            mm = px_to_mm([(x, y)], H)[0]
            state["last"] = ((x, y), (float(mm[0]), float(mm[1])))
            print(f"  pixel ({x:4d},{y:4d})  ->  "
                  f"X {mm[0]:7.1f} mm   Y {mm[1]:7.1f} mm")

    cv2.namedWindow(window)
    cv2.setMouseCallback(window, on_mouse)

    print("\nVERIFY")
    print("  The green grid is drawn from the calibration, spaced "
          f"{spacing} mm.")
    print("  Lay a ruler in the scene. If the lines match the ruler, the")
    print("  calibration is good. If they drift or skew, it is not.")
    print("  Click anywhere to read that point in millimetres. ESC quits.\n")

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        draw_mm_grid(frame, H_inv, width_mm, height_mm, spacing)

        if state["last"]:
            (px, py), (mx, my) = state["last"]
            cv2.circle(frame, (px, py), 7, (0, 0, 255), -1)
            cv2.putText(frame, f"X {mx:.1f}  Y {my:.1f} mm",
                        (px + 12, py + 6), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, (0, 0, 255), 2)

        cv2.putText(frame, "click = read mm    ESC = quit", (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow(window, frame)
        if (cv2.waitKey(1) & 0xFF) == 27:
            break


# ==========================================================================

def main():
    ap = argparse.ArgumentParser(description="Camera to millimetre calibration")
    ap.add_argument("--rect", nargs=2, type=float, metavar=("W", "H"),
                    help="measured rectangle size in mm, e.g. --rect 200 150")
    ap.add_argument("--verify", action="store_true",
                    help="skip calibration, use the saved one")
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--grid", type=float, default=25.0,
                    help="grid spacing in mm for the overlay")
    args = ap.parse_args()

    if not args.rect and not args.verify:
        raise SystemExit("give --rect W H to calibrate, or --verify to reuse")

    cap = open_camera(args.device, args.width, args.height)
    try:
        if args.verify:
            H, H_inv, w_mm, h_mm = load_calibration()
            print(f"loaded calibration for a {w_mm} x {h_mm} mm rectangle")
        else:
            w_mm, h_mm = args.rect
            frame = grab_still(cap)
            cv2.destroyWindow("camera")
            pts = collect_corners(frame)
            H, H_inv, errs = solve_homography(pts, w_mm, h_mm)

            print("\nreprojection error at the four corners:")
            for name, e in zip(CORNER_NAMES, errs):
                print(f"  {name:>18}  {e:6.3f} mm")
            if errs.max() > 1.0:
                print("\n  WARNING: this should be near zero for four points.")
                print("  Corners were probably clicked out of order, or two")
                print("  are nearly the same point.")
            save_calibration(H, w_mm, h_mm)

        verify(cap, H, H_inv, w_mm, h_mm, args.grid)
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
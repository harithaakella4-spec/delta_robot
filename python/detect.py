"""
Object detection -> millimetre coordinates.  (v2 - diagnostic)

Shape-agnostic detection of anything placed on the calibrated surface,
reported in millimetres via the homography from calibrate.py.

    python detect.py --device 1

v2 ANSWERS "WHY WAS THIS NOT DETECTED?"
    Every contour the camera finds is drawn. Accepted ones are green.
    REJECTED ones are red, labelled with the reason:

        too small / too big     failed the area filter
        outside region          centroid fell outside the calibrated
                                rectangle - this is the usual cause of
                                "it works on one side of the sheet only"

    The calibrated region itself is drawn as a blue outline. Anything
    outside that outline is discarded, because the homography is only
    trustworthy inside the rectangle it was fitted to. If the blue
    outline does not cover the area you are placing objects on, that is
    the problem - recalibrate over a larger rectangle.

WORKFLOW
    1. Clear the surface completely.
    2. Press 'b' to capture the empty background.
    3. Place objects.

KEYS
    b        capture background (surface must be EMPTY)
    s        save accepted detections to detections.json
    [ / ]    lower / raise threshold
    m        toggle mask view - shows what the camera actually separated
    r        toggle showing rejected contours
    ESC      quit

IF NOTHING IS DETECTED AT ALL
    Press 'm' and look at the mask. White blobs where your objects are
    means detection is working and a filter is rejecting them - the red
    labels will say which. A black mask means the objects are not
    separating from the background: lower the threshold with '[', or
    recapture the background with 'b' if the lighting has shifted.

IF ONLY SOME OBJECTS APPEAR
    Usually the area filter. A small screw can fall below --min-area and
    a large part above --max-area. Defaults are deliberately wide here;
    narrow them once you know your objects.
"""

import argparse
import json
import os

import cv2
import numpy as np

CALIB_FILE = "calibration.json"
OUT_FILE = "detections.json"


def load_calibration():
    if not os.path.exists(CALIB_FILE):
        raise SystemExit(f"{CALIB_FILE} not found - run calibrate.py first")
    with open(CALIB_FILE) as f:
        d = json.load(f)
    H = np.array(d["H"], dtype=np.float64)
    return H, np.linalg.inv(H), d["width_mm"], d["height_mm"]


def to_mm(pts, H):
    pts = np.asarray(pts, dtype=np.float32).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, H).reshape(-1, 2)


def to_px(pts, H_inv):
    pts = np.asarray(pts, dtype=np.float32).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, H_inv).reshape(-1, 2)


# ==========================================================================
# Detection
# ==========================================================================

def find_objects(frame, background, H, w_mm, h_mm,
                 thresh, min_area_mm2, max_area_mm2):
    """
    Returns (accepted, rejected, mask). Rejected entries carry the reason
    so the display can explain itself rather than silently dropping them.
    """
    diff = cv2.absdiff(background, frame)
    gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    _, mask = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    accepted, rejected = [], []

    for c in contours:
        if len(c) < 5:
            continue

        # Measure in millimetre space: under perspective a pixel-space
        # size is wrong by more the further it is from the lens centre.
        c_mm = to_mm(c.reshape(-1, 2), H).astype(np.float32)

        M = cv2.moments(c_mm)
        if M["m00"] == 0:
            continue
        cx, cy = M["m10"] / M["m00"], M["m01"] / M["m00"]
        area = abs(cv2.contourArea(c_mm))

        (_, _), (rw, rh), angle = cv2.minAreaRect(c_mm)
        length, width = max(rw, rh), min(rw, rh)
        if rw < rh:
            angle += 90.0
        angle = angle % 180.0

        obj = {
            "x_mm": round(float(cx), 2),
            "y_mm": round(float(cy), 2),
            "area_mm2": round(float(area), 1),
            "length_mm": round(float(length), 2),
            "width_mm": round(float(width), 2),
            "angle_deg": round(float(angle), 1),
            "contour_px": c,
        }

        if area < min_area_mm2:
            obj["reason"] = f"too small {area:.0f}mm2"
            rejected.append(obj)
        elif area > max_area_mm2:
            obj["reason"] = f"too big {area:.0f}mm2"
            rejected.append(obj)
        elif not (0 <= cx <= w_mm and 0 <= cy <= h_mm):
            obj["reason"] = f"outside region ({cx:.0f},{cy:.0f})"
            rejected.append(obj)
        else:
            accepted.append(obj)

    accepted.sort(key=lambda o: (o["y_mm"], o["x_mm"]))
    return accepted, rejected, mask


def flag_crowding(objects, clearance_mm):
    """Objects too close together to descend on safely - the effector
    plate has a footprint and can knock a neighbour."""
    for o in objects:
        o["crowded"] = False
    for i, a in enumerate(objects):
        for b in objects[i + 1:]:
            if np.hypot(a["x_mm"] - b["x_mm"],
                        a["y_mm"] - b["y_mm"]) < clearance_mm:
                a["crowded"] = b["crowded"] = True
    return objects


# ==========================================================================
# Display
# ==========================================================================

def centroid_px(contour):
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return None
    return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])


def draw_region(frame, H_inv, w_mm, h_mm):
    """Outline the calibrated rectangle. Objects outside it are dropped."""
    corners = to_px([(0, 0), (w_mm, 0), (w_mm, h_mm), (0, h_mm)], H_inv)
    cv2.polylines(frame, [np.int32(corners)], True, (255, 120, 0), 2)
    for (mx, my), p in zip([(0, 0), (w_mm, 0), (0, h_mm)], corners):
        cv2.putText(frame, f"({mx:.0f},{my:.0f})",
                    (int(p[0]) + 6, int(p[1]) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 120, 0), 1)


def draw(frame, accepted, rejected, show_rejected):
    if show_rejected:
        for o in rejected:
            cv2.drawContours(frame, [o["contour_px"]], -1, (0, 0, 255), 2)
            p = centroid_px(o["contour_px"])
            if p:
                cv2.putText(frame, o["reason"], (p[0] + 10, p[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    for i, o in enumerate(accepted):
        colour = (0, 165, 255) if o["crowded"] else (0, 255, 0)
        cv2.drawContours(frame, [o["contour_px"]], -1, colour, 2)
        p = centroid_px(o["contour_px"])
        if not p:
            continue
        cv2.drawMarker(frame, p, (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
        cv2.putText(frame, f"{i}", (p[0] + 10, p[1] - 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(frame, f"{o['x_mm']:.0f},{o['y_mm']:.0f}",
                    (p[0] + 10, p[1] + 2), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 2)
        cv2.putText(frame, f"{o['length_mm']:.0f}x{o['width_mm']:.0f} "
                           f"@{o['angle_deg']:.0f}",
                    (p[0] + 10, p[1] + 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (200, 200, 200), 1)


# ==========================================================================

def main():
    ap = argparse.ArgumentParser(description="Detect objects, report in mm")
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--thresh", type=int, default=30)
    ap.add_argument("--min-area", type=float, default=10.0,
                    help="smallest object to accept, mm^2")
    ap.add_argument("--max-area", type=float, default=40000.0,
                    help="largest object to accept, mm^2")
    ap.add_argument("--clearance", type=float, default=45.0,
                    help="spacing below which objects are flagged crowded, mm")
    args = ap.parse_args()

    H, H_inv, w_mm, h_mm = load_calibration()
    print(f"calibrated region: {w_mm:.0f} x {h_mm:.0f} mm")
    print("Objects outside the blue outline are discarded - if it does not")
    print("cover your working area, recalibrate over a bigger rectangle.\n")

    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        raise SystemExit(f"could not open camera {args.device}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    background = None
    thresh = args.thresh
    show_mask = False
    show_rejected = True
    accepted = []

    print("Clear the surface, then press 'b'.")
    print("b background | s save | [ ] threshold | m mask | r rejects | ESC\n")

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        display = frame.copy()
        draw_region(display, H_inv, w_mm, h_mm)

        if background is None:
            cv2.putText(display, "clear the surface, then press 'b'",
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            accepted, rejected, mask = find_objects(
                frame, background, H, w_mm, h_mm,
                thresh, args.min_area, args.max_area)
            accepted = flag_crowding(accepted, args.clearance)
            draw(display, accepted, rejected, show_rejected)
            cv2.putText(display,
                        f"{len(accepted)} ok, {len(rejected)} rejected   "
                        f"thresh {thresh}",
                        (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 255), 2)
            if show_mask:
                cv2.imshow("mask", mask)

        cv2.imshow("detect", display)
        k = cv2.waitKey(1) & 0xFF

        if k == 27:
            break
        elif k == ord("b"):
            background = frame.copy()
            print("background captured")
        elif k == ord("["):
            thresh = max(5, thresh - 5)
            print(f"threshold {thresh}")
        elif k == ord("]"):
            thresh = min(250, thresh + 5)
            print(f"threshold {thresh}")
        elif k == ord("m"):
            show_mask = not show_mask
            if not show_mask:
                cv2.destroyWindow("mask")
        elif k == ord("r"):
            show_rejected = not show_rejected
            print(f"show rejected: {show_rejected}")
        elif k == ord("s"):
            out = [{kk: vv for kk, vv in o.items() if kk != "contour_px"}
                   for o in accepted]
            with open(OUT_FILE, "w") as f:
                json.dump(out, f, indent=2)
            print(f"\nsaved {len(out)} objects to {OUT_FILE}")
            for i, o in enumerate(out):
                flag = "  CROWDED" if o["crowded"] else ""
                print(f"  {i}: X {o['x_mm']:7.1f}  Y {o['y_mm']:7.1f}  "
                      f"{o['length_mm']:5.1f} x {o['width_mm']:5.1f} mm  "
                      f"@ {o['angle_deg']:5.1f} deg{flag}")
            print()

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
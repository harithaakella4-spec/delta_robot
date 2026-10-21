"""
Delta Robot Digital Twin - live linked to hardware.

Drag the sliders, the on-screen robot updates, and the physical shoulders
follow. Single file: kinematics are embedded, so there is no import of the
old kinematics.py and no chance of running against its geometry bug.

    pip install pyserial matplotlib numpy

    python twin.py --mode sim
    python twin.py --mode arduino --port /dev/ttyACM0
    python twin.py --mode arduino --port COM3

Requires delta_angle_control.ino flashed on the Mega.
Close the Arduino IDE Serial Monitor first; only one program can hold
the port at a time.
"""

import argparse
import math
import time

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, Button, CheckButtons


# ==========================================================================
# GEOMETRY  -  Delta X "Puly S", values from Geometry.h
# ==========================================================================

RD_F  = 259.04    # base triangle side, mm
RD_E  = 120.00    # effector triangle side, mm
RD_RF = 130.00    # upper arm length, mm
RD_RE = 315.00    # lower arm (parallel rod) length, mm

# CRITICAL: these are INRADII, not circumradii. The Delta X firmware sets
# _y1_ = -0.5 * tan30 * RD_F, i.e. RD_F / (2*sqrt(3)) = 74.78 mm. The
# shoulder joints sit at the MIDPOINTS of the base triangle sides, not at
# the vertices. Using f/sqrt(3) here doubles both radii and throws the
# commanded angles out by up to 11 degrees while still looking plausible.
BASE_RADIUS     = RD_F / (2 * math.sqrt(3))   # 74.78 mm
EFFECTOR_RADIUS = RD_E / (2 * math.sqrt(3))   # 34.64 mm

# Arm placement around the base, degrees.
ARM_PHI = [0.0, 120.0, 240.0]

# Sign convention: +theta = arm tip ABOVE horizontal.
# Delta X Firmware uses the opposite. Its Puly S home of -38.5 deg is
# +38.5 deg here. Use to_deltax() at the boundary when you migrate.
def to_deltax(theta_deg):
    """Convert this file's angle convention to Delta X Firmware's."""
    return -theta_deg


def inverse_kinematics(x, y, z):
    """
    Cartesian effector position -> three shoulder angles in degrees.
    Returns None if the point is unreachable.

    Solves a*cos(t) + b*sin(t) = c per arm, taking the elbow-out branch.
    """
    angles = []
    for phi_deg in ARM_PHI:
        phi = math.radians(phi_deg)

        # Rotate the target into this arm's plane.
        x_loc = x * math.cos(phi) + y * math.sin(phi)
        y_loc = -x * math.sin(phi) + y * math.cos(phi)

        a = x_loc + EFFECTOR_RADIUS - BASE_RADIUS
        b = z
        c = (a * a + y_loc * y_loc + z * z + RD_RF ** 2 - RD_RE ** 2) / (2 * RD_RF)

        r_sq = a * a + b * b
        if r_sq == 0 or c * c > r_sq:
            return None

        dist = math.sqrt(r_sq)
        alpha = math.atan2(b, a)
        beta = math.acos(max(-1.0, min(1.0, c / dist)))

        angles.append(math.degrees(alpha + beta))   # elbow-out branch

    return tuple(angles)


def joint_positions(x, y, z, angles):
    """Base, elbow and wrist joint coordinates for a pose."""
    base, elbow, wrist = [], [], []
    for phi_deg, theta_deg in zip(ARM_PHI, angles):
        phi = math.radians(phi_deg)
        theta = math.radians(theta_deg)

        base.append((BASE_RADIUS * math.cos(phi),
                     BASE_RADIUS * math.sin(phi),
                     0.0))

        arm_x = BASE_RADIUS + RD_RF * math.cos(theta)
        elbow.append((arm_x * math.cos(phi),
                      arm_x * math.sin(phi),
                      RD_RF * math.sin(theta)))

        wrist.append((x + EFFECTOR_RADIUS * math.cos(phi),
                      y + EFFECTOR_RADIUS * math.sin(phi),
                      z))
    return base, elbow, wrist


# ==========================================================================
# SAFETY  -  must match ANGLE_MIN / ANGLE_MAX in delta_angle_control.ino
# Duplicated on purpose: a GUI bug cannot drive past a limit the firmware
# would have caught, and vice versa.
# ==========================================================================

ANGLE_MIN = -45.0
ANGLE_MAX = 45.0

LIVE_INTERVAL = 0.12      # min seconds between commands in live mode
PATH_STEP_MM = 4.0        # interpolation granularity for Send


def in_limits(angles):
    return angles is not None and all(ANGLE_MIN <= a <= ANGLE_MAX for a in angles)


def interpolate(start, end, step_mm=PATH_STEP_MM):
    """Straight-line Cartesian path. A delta sweeps an arc between two
    poses unless you interpolate the path in Cartesian space."""
    dist = math.dist(start, end)
    n = max(1, int(dist / step_mm))
    return [tuple(s + (e - s) * i / n for s, e in zip(start, end))
            for i in range(1, n + 1)]


# ==========================================================================
# Hardware interfaces
# ==========================================================================

class SimRobot:
    """Stand-in so the GUI runs with nothing plugged in."""

    def __init__(self):
        self.angles = (0.0, 0.0, 0.0)

    def connect(self):
        print("[SIM] connected")
        return True

    def move_joints(self, a, b, c):
        self.angles = (a, b, c)
        return True

    def set_pose(self, a, b, c):
        self.angles = (a, b, c)
        return True

    def release(self):
        print("[SIM] drivers disabled")
        return True

    def close(self):
        pass


class SerialRobot:
    """
    Talks to delta_angle_control.ino.

    Three deliberate differences from a naive serial wrapper:
      - a failed handshake is a failed connect, not a silent success
      - replies are read until OK/ERR against a deadline, so a long move
        cannot leave the stream one line out of sync forever
      - the input buffer is drained before every command
    """

    def __init__(self, port, baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None

    def connect(self):
        try:
            import serial
        except ImportError:
            print("pyserial missing.  pip install pyserial")
            return False

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1.0)
        except Exception as e:
            print(f"could not open {self.port}: {e}")
            return False

        time.sleep(2.5)                     # bootloader reset delay
        self.ser.reset_input_buffer()

        self.ser.write(b"?\n")
        reply = self._read_until(lambda s: s.startswith("A"), deadline=3.0)
        if reply is None:
            print("no response from Arduino. Wrong port, sketch not flashed, "
                  "or the Serial Monitor is still holding the port.")
            self.ser.close()
            self.ser = None
            return False

        print(f"connected on {self.port}  |  {reply}")
        return True

    def _read_until(self, predicate, deadline):
        end = time.time() + deadline
        while time.time() < end:
            line = self.ser.readline().decode(errors="replace").strip()
            if line and predicate(line):
                return line
        return None

    def _command(self, cmd, deadline=30.0):
        if self.ser is None:
            return False
        self.ser.reset_input_buffer()
        self.ser.write((cmd + "\n").encode())
        reply = self._read_until(
            lambda s: s.startswith("OK") or s.startswith("ERR"),
            deadline=deadline)
        if reply is None:
            print(f"timeout waiting for reply to: {cmd}")
            return False
        if reply.startswith("ERR"):
            print(f"{cmd}  ->  {reply}")
            return False
        return True

    def move_joints(self, a, b, c):
        return self._command(f"M1 A{a:.2f} B{b:.2f} C{c:.2f}")

    def set_pose(self, a, b, c):
        return self._command(f"M2 A{a:.2f} B{b:.2f} C{c:.2f}")

    def release(self):
        return self._command("M0", deadline=3.0)

    def close(self):
        if self.ser is not None:
            try:
                self._command("M0", deadline=2.0)   # leave motors de-energised
            finally:
                self.ser.close()
                self.ser = None


# ==========================================================================
# Main
# ==========================================================================

def main():
    ap = argparse.ArgumentParser(description="Delta Robot Digital Twin")
    ap.add_argument("--mode", default="sim", choices=["sim", "arduino"])
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--start-z", type=float, default=-280.0)
    args = ap.parse_args()

    print(f"base radius {BASE_RADIUS:.2f} mm   effector radius "
          f"{EFFECTOR_RADIUS:.2f} mm   limits [{ANGLE_MIN}, {ANGLE_MAX}] deg")

    robot = SimRobot() if args.mode == "sim" else SerialRobot(args.port)
    if not robot.connect():
        print("aborting.")
        return

    start_pos = (0.0, 0.0, args.start_z)
    start_angles = inverse_kinematics(*start_pos)

    if not in_limits(start_angles):
        print(f"start pose {start_pos} needs angles {start_angles}, "
              f"outside [{ANGLE_MIN}, {ANGLE_MAX}]. Try a different --start-z.")
        robot.close()
        return

    print(f"start pose {start_pos} -> "
          f"{tuple(round(a, 2) for a in start_angles)} deg")
    robot.move_joints(*start_angles)

    state = {"pos": start_pos, "last_send": 0.0}

    # ---- Figure ---------------------------------------------------------
    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")
    plt.subplots_adjust(left=0.05, bottom=0.30)

    ax.set_xlim(-300, 300)
    ax.set_ylim(-300, 300)
    ax.set_zlim(-450, 100)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.set_title(f"Delta Digital Twin  [{args.mode.upper()}]")

    base_pts, _, _ = joint_positions(0, 0, args.start_z, (0, 0, 0))
    bx = [p[0] for p in base_pts] + [base_pts[0][0]]
    by = [p[1] for p in base_pts] + [base_pts[0][1]]
    ax.plot(bx, by, [0] * 4, "k-", lw=2, label="base")

    upper = [ax.plot([], [], [], "-", color="#2a8", lw=4)[0] for _ in range(3)]
    lower = [ax.plot([], [], [], "-", color="#999", lw=2)[0] for _ in range(3)]
    eff_line, = ax.plot([], [], [], "b-", lw=2, label="effector")
    target_dot, = ax.plot([], [], [], "rx", ms=9)
    ax.legend(loc="upper right")

    status = ax.text2D(0.02, 0.93, "", transform=ax.transAxes,
                       family="monospace", fontsize=9)

    def redraw(pos):
        angles = inverse_kinematics(*pos)
        if angles is None:
            status.set_text("UNREACHABLE")
            status.set_color("crimson")
            return None

        _, elbow, wrist = joint_positions(*pos, angles)
        for i in range(3):
            upper[i].set_data([base_pts[i][0], elbow[i][0]],
                              [base_pts[i][1], elbow[i][1]])
            upper[i].set_3d_properties([base_pts[i][2], elbow[i][2]])
            lower[i].set_data([elbow[i][0], wrist[i][0]],
                              [elbow[i][1], wrist[i][1]])
            lower[i].set_3d_properties([elbow[i][2], wrist[i][2]])

        ex = [p[0] for p in wrist] + [wrist[0][0]]
        ey = [p[1] for p in wrist] + [wrist[0][1]]
        ez = [p[2] for p in wrist] + [wrist[0][2]]
        eff_line.set_data(ex, ey)
        eff_line.set_3d_properties(ez)

        ok = in_limits(angles)
        status.set_text(
            f"XYZ  {pos[0]:7.1f} {pos[1]:7.1f} {pos[2]:7.1f}\n"
            f"ABC  {angles[0]:7.2f} {angles[1]:7.2f} {angles[2]:7.2f}"
            f"{'' if ok else '   OUT OF LIMITS'}")
        status.set_color("black" if ok else "crimson")
        return angles

    # ---- Controls -------------------------------------------------------
    s_x = Slider(plt.axes([0.12, 0.20, 0.62, 0.03]), "X", -180, 180,
                 valinit=start_pos[0])
    s_y = Slider(plt.axes([0.12, 0.15, 0.62, 0.03]), "Y", -180, 180,
                 valinit=start_pos[1])
    s_z = Slider(plt.axes([0.12, 0.10, 0.62, 0.03]), "Z", -380, -180,
                 valinit=start_pos[2])

    chk_live = CheckButtons(plt.axes([0.80, 0.14, 0.13, 0.07]),
                            ["live"], [False])
    b_send = Button(plt.axes([0.80, 0.09, 0.13, 0.045]), "Send")
    b_rel = Button(plt.axes([0.80, 0.035, 0.13, 0.045]), "Release")

    def live_on():
        return chk_live.get_status()[0]

    def on_slider(_val):
        pos = (s_x.val, s_y.val, s_z.val)
        target_dot.set_data([pos[0]], [pos[1]])
        target_dot.set_3d_properties([pos[2]])
        angles = redraw(pos)

        if live_on() and in_limits(angles):
            now = time.time()
            if now - state["last_send"] >= LIVE_INTERVAL:
                state["last_send"] = now
                if robot.move_joints(*angles):
                    state["pos"] = pos
        fig.canvas.draw_idle()

    s_x.on_changed(on_slider)
    s_y.on_changed(on_slider)
    s_z.on_changed(on_slider)

    def on_send(_event):
        """Interpolated straight-line move from the robot's position to
        the slider target. Stops at the first invalid waypoint."""
        target = (s_x.val, s_y.val, s_z.val)
        for point in interpolate(state["pos"], target):
            angles = inverse_kinematics(*point)
            if not in_limits(angles):
                print(f"path blocked at {tuple(round(v,1) for v in point)}"
                      f" -> {angles}")
                break
            if not robot.move_joints(*angles):
                print("move rejected, stopping.")
                break
            state["pos"] = point
            redraw(point)
            fig.canvas.draw()
            fig.canvas.flush_events()

    def on_release(_event):
        robot.release()
        print("drivers disabled. Redo the startup ritual before moving again.")

    b_send.on_clicked(on_send)
    b_rel.on_clicked(on_release)

    redraw(start_pos)
    target_dot.set_data([start_pos[0]], [start_pos[1]])
    target_dot.set_3d_properties([start_pos[2]])

    try:
        plt.show()
    finally:
        robot.close()


if __name__ == "__main__":
    main()
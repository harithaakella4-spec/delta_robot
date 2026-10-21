"""
Delta Robot - jog, digital twin, workspace analysis and Cartesian sweep.

    pip install pyserial matplotlib numpy

    python twin_jog.py --mode sim
    python twin_jog.py --mode arduino --port COM6
    python twin_jog.py --mode arduino --port COM6 --offsets 0,2.1,0

Requires delta_homing.ino v6 on the Mega (J1/J2/J3/JA/M1/G28/M0/? commands).
Close the Arduino Serial Monitor first - only one program can hold the port.

JOG
    Buttons move one shoulder, or all three, by a number of degrees.
    Angles are read back after every move, so the twin shows the robot's
    own reported state rather than an assumption.

WORKSPACE
    Scans the reachable volume and reports, for each height, the largest
    circle in which EVERY point is reachable.

    That inscribed circle is the number that matters. The reachable
    region at a given height is a rounded triangle, not a disc, because
    the arms sit 120 degrees apart. Quoting maximum radius overstates the
    usable area badly - that is reach toward one arm only.

SWEEP  (R30 / R50 / R80 / R120)
    Traces a circle at --sweep-z so you can watch the real working area.
    Python runs the inverse kinematics and sends joint angles with M1;
    the firmware needs no Cartesian command.

    Every point is validated against the angle limits BEFORE anything
    moves. If one point is out of reach the whole sweep is refused, so
    the machine never discovers a limit by driving into it.

    START AT R30. Watch a full circle, check the effector stays level,
    then grow. Sagging or tilting part-way round means the kinematics and
    the machine disagree - stop and check link lengths and the arm 2
    offset rather than increasing the radius.

    STOP halts a path at any point, including mid-motion. The host sends
    a single '!' which the firmware checks between step batches; it stops
    where it is, keeps the drivers energised so the arms hold, and keeps
    counting steps - so the position stays known and no re-home is
    needed after an abort.

SPIRAL  (--spiral-radius, --spiral-turns)
    Runs an Archimedean spiral from the centre out to the chosen radius.
    A better drivetrain test than any single circle: it crosses every
    radius and every angle in one continuous path, so load on the arms
    climbs steadily outward instead of sitting at one value. A belt that
    slips only at a particular reach, or only at one point in its travel,
    will show up here when a fixed-radius circle might miss it.

ARM OFFSETS  (--offsets 0,2.1,0)
    If one endstop fires at a different arm angle than the others,
    homing labels all three the same and that arm ends up displaced.
    Measure the difference with G28 / M1 A10 B10 C10 / G28, comparing
    trigger travel across axes, and correct it here.

    Only compensate a difference that REPEATS. A figure that wanders
    run to run is belt slip or a marginal switch, and calibrating it
    away just encodes a moving target.

LIMITS
    --angle-min and --angle-max gate every move. They have not been
    measured on this machine with the linkage attached, so treat the
    workspace figures as the shape of the answer, not the answer.
"""

import argparse
import math
import re
import time

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button


# ==========================================================================
# GEOMETRY - Delta X "Puly S" (Geometry.h)
# Radii are INRADII: the firmware uses 0.5*tan30*side = side/(2*sqrt3).
# ==========================================================================

RD_F, RD_E, RD_RF, RD_RE = 259.04, 120.00, 130.00, 315.00

BASE_RADIUS     = RD_F / (2 * math.sqrt(3))    # 74.78 mm
EFFECTOR_RADIUS = RD_E / (2 * math.sqrt(3))    # 34.64 mm

ARM_PHI = [0.0, 120.0, 240.0]

# Angle convention: +theta = arm tip ABOVE horizontal, 0 = level with base.


def to_deltax(theta_deg):
    """Delta X Firmware defines +theta as arm-DOWN. Convert at the boundary."""
    return -theta_deg


# ---------------------------- kinematics ---------------------------------

def inverse_kinematics(x, y, z):
    """Effector position -> three shoulder angles, or None if unreachable."""
    angles = []
    for phi_deg in ARM_PHI:
        phi = math.radians(phi_deg)
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
        angles.append(math.degrees(alpha + beta))     # elbow-out branch
    return tuple(angles)


def elbow_positions(angles_deg):
    """Shoulder and elbow coordinates for three joint angles."""
    shoulders, elbows = [], []
    for phi_deg, theta_deg in zip(ARM_PHI, angles_deg):
        phi, theta = math.radians(phi_deg), math.radians(theta_deg)
        shoulders.append((BASE_RADIUS * math.cos(phi),
                          BASE_RADIUS * math.sin(phi), 0.0))
        arm_x = BASE_RADIUS + RD_RF * math.cos(theta)
        elbows.append((arm_x * math.cos(phi),
                       arm_x * math.sin(phi),
                       RD_RF * math.sin(theta)))
    return shoulders, elbows


def forward_kinematics(angles_deg):
    """
    Three joint angles -> effector centre, or None if the pose does not
    close. Each lower arm puts the effector on a sphere of radius RD_RE
    about (elbow - effector_offset); the answer is where they meet.
    """
    _, elbows = elbow_positions(angles_deg)

    centres = []
    for phi_deg, e in zip(ARM_PHI, elbows):
        phi = math.radians(phi_deg)
        centres.append((e[0] - EFFECTOR_RADIUS * math.cos(phi),
                        e[1] - EFFECTOR_RADIUS * math.sin(phi),
                        e[2]))

    p1, p2, p3 = [tuple(map(float, c)) for c in centres]

    ex = [p2[i] - p1[i] for i in range(3)]
    d = math.sqrt(sum(v * v for v in ex))
    if d == 0:
        return None
    ex = [v / d for v in ex]

    t = [p3[i] - p1[i] for i in range(3)]
    i_val = sum(ex[k] * t[k] for k in range(3))
    ey = [t[k] - i_val * ex[k] for k in range(3)]
    ny = math.sqrt(sum(v * v for v in ey))
    if ny == 0:
        return None
    ey = [v / ny for v in ey]

    ez = [ex[1] * ey[2] - ex[2] * ey[1],
          ex[2] * ey[0] - ex[0] * ey[2],
          ex[0] * ey[1] - ex[1] * ey[0]]

    j = sum(ey[k] * t[k] for k in range(3))
    x = d / 2.0
    if j == 0:
        return None
    y = (i_val * i_val + j * j) / (2 * j) - (i_val / j) * x

    zz = RD_RE ** 2 - x * x - y * y
    if zz < 0:
        return None
    z = math.sqrt(zz)

    lower = [p1[k] + x * ex[k] + y * ey[k] - z * ez[k] for k in range(3)]
    upper = [p1[k] + x * ex[k] + y * ey[k] + z * ez[k] for k in range(3)]
    return tuple(lower if lower[2] < upper[2] else upper)


# ==========================================================================
# WORKSPACE
# ==========================================================================

def reachable(x, y, z, angle_min, angle_max):
    a = inverse_kinematics(x, y, z)
    return a is not None and all(angle_min <= v <= angle_max for v in a)


def usable_radius_at_z(z, angle_min, angle_max,
                       max_r=400.0, step=2.0, n_angles=36):
    """Largest radius at height z where EVERY point on the circle is
    reachable - a circle you can place objects anywhere inside."""
    best = 0.0
    r = step
    while r <= max_r:
        for k in range(n_angles):
            th = 2.0 * math.pi * k / n_angles
            if not reachable(r * math.cos(th), r * math.sin(th), z,
                             angle_min, angle_max):
                return best
        best = r
        r += step
    return best


def max_radius_at_z(z, angle_min, angle_max,
                    max_r=400.0, step=2.0, n_angles=36):
    """Furthest reachable point at this height in ANY direction. Always
    larger than the usable radius - shown for contrast, not for use."""
    best = 0.0
    r = step
    while r <= max_r:
        if any(reachable(r * math.cos(2 * math.pi * k / n_angles),
                         r * math.sin(2 * math.pi * k / n_angles),
                         z, angle_min, angle_max)
               for k in range(n_angles)):
            best = r
        r += step
    return best


def workspace_profile(angle_min, angle_max,
                      z_lo=-450.0, z_hi=-80.0, z_step=5.0):
    rows = []
    z = z_lo
    while z <= z_hi:
        u = usable_radius_at_z(z, angle_min, angle_max)
        if u > 0:
            rows.append({"z": z, "usable_r": u,
                         "max_r": max_radius_at_z(z, angle_min, angle_max)})
        z += z_step
    return rows


def report_workspace(angle_min, angle_max):
    print(f"\n=== WORKSPACE  (shoulder limits {angle_min} to {angle_max} deg) ===")
    print("These limits are the assumption the whole result rests on.")
    print("Measure them on the machine before trusting any number here.\n")

    rows = workspace_profile(angle_min, angle_max)
    if not rows:
        print("  nothing reachable within these limits.")
        return [], None

    print(f"  {'Z (mm)':>8}  {'usable dia':>11}  {'max dia':>9}")
    print(f"  {'-'*8}  {'-'*11}  {'-'*9}")
    for r in rows[::4]:
        print(f"  {r['z']:8.0f}  {2*r['usable_r']:9.0f} mm  "
              f"{2*r['max_r']:7.0f} mm")

    best = max(rows, key=lambda r: r["usable_r"])
    print(f"\n  Z range with any reach : {min(r['z'] for r in rows):.0f} to "
          f"{max(r['z'] for r in rows):.0f} mm")
    print(f"  Best pick plane        : Z = {best['z']:.0f} mm")
    print(f"  Usable diameter there  : {2*best['usable_r']:.0f} mm")
    print(f"  (max reach there is {2*best['max_r']:.0f} mm, toward the arms only)")
    print("\n  Delta X quote 170 mm for this geometry - deliberately")
    print("  conservative, since accuracy falls off near the edge where")
    print("  the lower arms approach coplanar.\n")
    return rows, best


# ==========================================================================
# SWEEP PATH
# ==========================================================================

def circle_points(z, radius, n):
    return [(radius * math.cos(2 * math.pi * k / n),
             radius * math.sin(2 * math.pi * k / n),
             z) for k in range(n + 1)]


def spiral_points(z, r_max, turns, n):
    """
    Archimedean spiral from the centre out to r_max at constant height.

    A better workspace test than any single circle: it crosses every
    radius and every angle in one continuous path, so a drivetrain
    problem that only appears at a particular reach or a particular
    belt position has nowhere to hide. Load on the arms climbs steadily
    outward rather than sitting at one value.
    """
    pts = []
    for k in range(n + 1):
        f = k / n                                  # 0 -> 1
        r = r_max * f
        th = 2.0 * math.pi * turns * f
        pts.append((r * math.cos(th), r * math.sin(th), z))
    return pts


def conical_spiral_points(z_start, z_end, r_max, turns, n):
    """
    Spiral outward from the centre while climbing in Z - a cone traced
    from its apex.

    Every move this machine has made so far has been at constant height.
    This is the first path that drives all three axes through genuinely
    different ratios at once, and the first test of whether the geometry
    holds up away from a single plane. It also walks the effector through
    the part of the workspace that narrows as it rises, which is where an
    out-of-reach point is most likely to appear.
    """
    pts = []
    for k in range(n + 1):
        f = k / n                                  # 0 -> 1
        r = r_max * f
        z = z_start + (z_end - z_start) * f
        th = 2.0 * math.pi * turns * f
        pts.append((r * math.cos(th), r * math.sin(th), z))
    return pts


def plan_path(points, angle_min, angle_max, offsets):
    """
    Cartesian points -> joint angles, refusing the whole path if any point
    is out of reach. Returns (planned, rejected). Checking up front means
    the machine never finds a limit by hitting it.
    """
    out, bad = [], []
    for p in points:
        a = inverse_kinematics(*p)
        if a is None:
            bad.append((p, "unreachable"))
        elif not all(angle_min <= v <= angle_max for v in a):
            bad.append((p, f"limits {tuple(round(v,1) for v in a)}"))
        else:
            out.append((p, a, tuple(a[i] + offsets[i] for i in range(3))))
    return out, bad


# ==========================================================================
# Interfaces
# ==========================================================================

ANGLE_RE = re.compile(r"A(-?\d+\.?\d*)\s+B(-?\d+\.?\d*)\s+C(-?\d+\.?\d*)")


class SimRobot:
    def __init__(self):
        self.angles = [0.0, 0.0, 0.0]

    def connect(self):
        print("[SIM] connected")
        return True

    def jog(self, axis, delta):
        if axis is None:
            self.angles = [a + delta for a in self.angles]
        else:
            self.angles[axis] += delta
        return True

    def move_angles(self, a, b, c):
        self.angles = [a, b, c]
        return True

    def queue_clear(self):
        self._q = []
        return True

    def queue_point(self, a, b, c):
        self._q = getattr(self, "_q", [])
        self._q.append((a, b, c))
        return True

    def abort(self):
        print("[SIM] abort")
        return True

    def queue_run(self, deadline=300.0, poll=None):
        q = getattr(self, "_q", [])
        if poll is not None:
            poll()
        if q:
            self.angles = list(q[-1])
        print(f"[SIM] executed {len(q)} queued points as one motion")
        self._q = []
        return True

    @property
    def max_queue(self):
        return 80

    def home(self):
        self.angles = [33.5, 33.5, 33.5]
        print("[SIM] homed")
        return True

    def release(self):
        print("[SIM] drivers off")
        return True

    def read_angles(self):
        return tuple(self.angles)

    def close(self):
        pass


class SerialRobot:
    def __init__(self, port, baud=115200):
        self.port, self.baud, self.ser = port, baud, None

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

        time.sleep(2.5)
        self.ser.reset_input_buffer()
        self.ser.write(b"?\n")
        if self._read_until(lambda s: ANGLE_RE.search(s), 3.0) is None:
            print("no response. Wrong port, sketch not flashed, or the "
                  "Serial Monitor still holds the port.")
            self.ser.close()
            self.ser = None
            return False
        print(f"connected on {self.port}")
        return True

    def _read_until(self, pred, deadline):
        end = time.time() + deadline
        while time.time() < end:
            line = self.ser.readline().decode(errors="replace").strip()
            if line:
                print(f"  < {line}")
                if pred(line):
                    return line
        return None

    def _cmd(self, text, deadline=60.0):
        if self.ser is None:
            return None
        self.ser.reset_input_buffer()
        self.ser.write((text + "\n").encode())
        return self._read_until(
            lambda s: s.startswith("OK") or s.startswith("ERR"), deadline)

    def jog(self, axis, delta):
        cmd = f"JA {delta:g}" if axis is None else f"J{axis + 1} {delta:g}"
        r = self._cmd(cmd)
        return bool(r and r.startswith("OK"))

    def move_angles(self, a, b, c):
        r = self._cmd(f"M1 A{a:.3f} B{b:.3f} C{c:.3f}")
        if r and r.startswith("OK"):
            return True
        print(f"  move refused: {r}")
        return False

    # ---- path queue: send every point, then run them as ONE motion ----

    def queue_clear(self):
        return bool(self._cmd("QC", deadline=5.0))

    def queue_point(self, a, b, c):
        r = self._cmd(f"Q A{a:.3f} B{b:.3f} C{c:.3f}", deadline=5.0)
        if r and r.startswith("OK"):
            return True
        print(f"  queue refused: {r}")
        return False

    def abort(self):
        """
        Send the mid-path abort. Written straight to the port rather than
        through _cmd: the firmware is mid-move and the normal
        request/response handshake does not apply.
        """
        if self.ser is None:
            return False
        try:
            self.ser.write(b"!\n")
            self.ser.flush()
            return True
        except Exception as e:
            print(f"  abort failed: {e}")
            return False

    def queue_run(self, deadline=300.0, poll=None):
        """
        Execute the queue without blocking the caller.

        readline() would sit on the port for the whole move, which freezes
        the GUI event loop - so the STOP button could never be clicked
        even though it existed. This polls instead and calls poll() every
        20 ms so the interface stays live and a click can land.
        """
        if self.ser is None:
            return False
        self.ser.reset_input_buffer()
        self.ser.write(b"X\n")

        buf = b""
        end = time.time() + deadline
        while time.time() < end:
            if poll is not None:
                poll()
            if self.ser.in_waiting:
                buf += self.ser.read(self.ser.in_waiting)
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode(errors="replace").strip()
                    if text:
                        print(f"  < {text}")
                        if text.startswith("OK"):
                            return True
                        if text.startswith("ERR"):
                            return False
            time.sleep(0.02)

        print("  timeout waiting for the path to finish")
        return False

    @property
    def max_queue(self):
        return 80

    def home(self):
        r = self._cmd("G28", deadline=180.0)
        return bool(r and r.startswith("OK"))

    def release(self):
        return bool(self._cmd("M0", deadline=5.0))

    def read_angles(self):
        if self.ser is None:
            return None
        self.ser.reset_input_buffer()
        self.ser.write(b"?\n")
        line = self._read_until(lambda s: ANGLE_RE.search(s), 3.0)
        if line is None:
            return None
        return tuple(float(g) for g in ANGLE_RE.search(line).groups())

    def close(self):
        if self.ser:
            try:
                self._cmd("M0", deadline=2.0)
            finally:
                self.ser.close()
                self.ser = None


# ==========================================================================
# GUI
# ==========================================================================

def main():
    ap = argparse.ArgumentParser(description="Delta jog, twin, workspace, sweep")
    ap.add_argument("--mode", default="sim", choices=["sim", "arduino"])
    ap.add_argument("--port", default="COM6")
    ap.add_argument("--angle-min", type=float, default=-60.0)
    ap.add_argument("--angle-max", type=float, default=40.0)
    ap.add_argument("--sweep-z", type=float, default=-280.0,
                    help="height of the sweep circle, mm")
    ap.add_argument("--sweep-points", type=int, default=36)
    ap.add_argument("--spiral-radius", type=float, default=80.0,
                    help="outer radius of the spiral, mm")
    ap.add_argument("--spiral-turns", type=float, default=3.0,
                    help="how many revolutions from centre to the rim")
    ap.add_argument("--spiral-points", type=int, default=76,
                    help="points in the spiral; firmware queue holds 80")
    ap.add_argument("--spiral-rise", type=float, default=40.0,
                    help="height change for the conical spirals, mm. The "
                         "workspace narrows as the effector rises, so a "
                         "large rise at a large radius will be refused.")
    ap.add_argument("--offsets", default="0,0,0",
                    help="per-axis correction in degrees, e.g. 0,2.1,0")
    ap.add_argument("--workspace", action="store_true",
                    help="print the workspace report and exit")
    args = ap.parse_args()

    offsets = [float(v) for v in args.offsets.split(",")]
    if len(offsets) != 3:
        raise SystemExit("--offsets needs three values")

    if args.workspace:
        report_workspace(args.angle_min, args.angle_max)
        return

    robot = SimRobot() if args.mode == "sim" else SerialRobot(args.port)
    if not robot.connect():
        return

    print(f"offsets {offsets}   sweep Z {args.sweep_z:.0f} mm   "
          f"limits {args.angle_min} to {args.angle_max} deg")

    angles = list(robot.read_angles() or (0.0, 0.0, 0.0))
    state = {"abort": False, "busy": False}

    fig = plt.figure(figsize=(11, 9))
    ax = fig.add_subplot(111, projection="3d")
    plt.subplots_adjust(left=0.02, right=0.98, bottom=0.50, top=0.98)

    ax.set_xlim(-300, 300)
    ax.set_ylim(-300, 300)
    ax.set_zlim(-450, 150)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")

    shoulders, _ = elbow_positions([0, 0, 0])
    bx = [p[0] for p in shoulders] + [shoulders[0][0]]
    by = [p[1] for p in shoulders] + [shoulders[0][1]]
    ax.plot(bx, by, [0] * 4, "k-", lw=2)

    for p in shoulders:                      # the 0 degree reference plane
        ax.plot([p[0], p[0] * 3], [p[1], p[1] * 3], [0, 0],
                color="#bbb", ls=":", lw=1)

    upper = [ax.plot([], [], [], "-", color="#2a8", lw=5)[0] for _ in range(3)]
    lower = [ax.plot([], [], [], "-", color="#bbb", lw=2)[0] for _ in range(3)]
    eff_line, = ax.plot([], [], [], "b-", lw=2)
    disc, = ax.plot([], [], [], "-", color="#c60", lw=2)
    trail, = ax.plot([], [], [], "-", color="#06c", lw=1)
    labels = [ax.text(0, 0, 0, "", fontsize=8) for _ in range(3)]

    status = ax.text2D(0.02, 0.95, "", transform=ax.transAxes,
                       family="monospace", fontsize=10)

    def redraw():
        sh, el = elbow_positions(angles)
        for i in range(3):
            upper[i].set_data([sh[i][0], el[i][0]], [sh[i][1], el[i][1]])
            upper[i].set_3d_properties([sh[i][2], el[i][2]])
            labels[i].set_position((el[i][0], el[i][1]))
            labels[i].set_3d_properties(el[i][2], zdir=None)
            labels[i].set_text(f" T{i+1} {angles[i]:.1f}")

        eff = forward_kinematics(angles)
        if eff is None:
            for i in range(3):
                lower[i].set_data([], [])
                lower[i].set_3d_properties([])
            eff_line.set_data([], [])
            eff_line.set_3d_properties([])
            extra = "lower arms: pose does not close"
        else:
            wrists = []
            for phi_deg in ARM_PHI:
                phi = math.radians(phi_deg)
                wrists.append((eff[0] + EFFECTOR_RADIUS * math.cos(phi),
                               eff[1] + EFFECTOR_RADIUS * math.sin(phi),
                               eff[2]))
            for i in range(3):
                lower[i].set_data([el[i][0], wrists[i][0]],
                                  [el[i][1], wrists[i][1]])
                lower[i].set_3d_properties([el[i][2], wrists[i][2]])
            ex = [w[0] for w in wrists] + [wrists[0][0]]
            ey = [w[1] for w in wrists] + [wrists[0][1]]
            ez = [w[2] for w in wrists] + [wrists[0][2]]
            eff_line.set_data(ex, ey)
            eff_line.set_3d_properties(ez)
            extra = f"effector  X{eff[0]:7.1f} Y{eff[1]:7.1f} Z{eff[2]:7.1f}"

        status.set_text(
            f"T1 {angles[0]:7.2f}   T2 {angles[1]:7.2f}   T3 {angles[2]:7.2f}  deg\n"
            f"{extra}")
        fig.canvas.draw_idle()

    def sync():
        got = robot.read_angles()
        if got:
            angles[:] = list(got)
        redraw()

    # ---- sweep ---------------------------------------------------------

    def run_spiral(r_max):
        """Centre -> r_max spiral. Queue depth caps the point count."""
        n = min(args.spiral_points, 76)
        pts = spiral_points(args.sweep_z, r_max, args.spiral_turns, n)
        run_path(pts, f"SPIRAL to R{r_max:.0f} "
                      f"({args.spiral_turns} turns, {n} points)")

    def run_cone(r_max, rise):
        """Spiral out to r_max while climbing (rise>0) or dropping."""
        n = min(args.spiral_points, 76)
        z0 = args.sweep_z
        z1 = args.sweep_z + rise
        pts = conical_spiral_points(z0, z1, r_max, args.spiral_turns, n)
        direction = "UP" if rise > 0 else "DOWN"
        run_path(pts, f"SPIRAL {direction} to R{r_max:.0f} "
                      f"(Z {z0:.0f} -> {z1:.0f}, {n} points)")

    def run_sweep(radius):
        pts = circle_points(args.sweep_z, radius, args.sweep_points)
        run_path(pts, f"SWEEP radius {radius:.0f} mm")

    def run_path(pts, label):
        if state["busy"]:
            print("already moving")
            return

        path, bad = plan_path(pts, args.angle_min, args.angle_max, offsets)

        if bad:
            print(f"\nREFUSED ({label}) - {len(bad)} of {len(pts)} "
                  f"points out of reach:")
            for p, why in bad[:4]:
                print(f"  ({p[0]:7.1f},{p[1]:7.1f},{p[2]:7.1f})  {why}")
            if len(bad) > 4:
                print(f"  ... and {len(bad)-4} more")
            print("Reduce the radius or change --sweep-z. Nothing moved.\n")
            return

        lo = [min(a[i] for _, a, _ in path) for i in range(3)]
        hi = [max(a[i] for _, a, _ in path) for i in range(3)]
        print(f"\n=== {label} at Z {args.sweep_z:.0f} mm ===")
        print(f"all {len(path)} points reachable. Joint range:")
        for i in range(3):
            print(f"  THETA{i+1}: {lo[i]:7.2f} to {hi[i]:7.2f} deg")

        centre = inverse_kinematics(0.0, 0.0, args.sweep_z)
        if centre is None:
            print("centre unreachable - change --sweep-z\n")
            return

        state["busy"] = True
        state["abort"] = False
        state["abort_sent"] = False
        try:
            print("moving to centre first")
            if not robot.move_angles(*[centre[i] + offsets[i]
                                       for i in range(3)]):
                return
            sync()

            # Draw the whole planned path up front. The machine runs it as
            # one continuous motion, so there is no per-point feedback to
            # animate - the trail shows what it is about to do.
            trail.set_data([p[0] for p, _, _ in path],
                           [p[1] for p, _, _ in path])
            trail.set_3d_properties([p[2] for p, _, _ in path])
            redraw()
            fig.canvas.flush_events()

            # Queue every point, then execute. Sent one at a time, each
            # point would be its own accelerate-decelerate cycle and the
            # circle would stutter; queued, the ramp spans the whole path.
            if len(path) > robot.max_queue:
                print(f"path is {len(path)} points, queue holds "
                      f"{robot.max_queue}. Lower --sweep-points.\n")
                return

            robot.queue_clear()
            for p, a, sent in path:
                if state["abort"]:
                    print("ABORTED before motion - nothing moved\n")
                    robot.queue_clear()
                    return
                if not robot.queue_point(*sent):
                    robot.queue_clear()
                    return
                fig.canvas.flush_events()

            print(f"queued {len(path)} points, executing as one motion...")
            print("STOP will halt it mid-path.")

            def poll():
                """
                Runs every 20 ms while the path executes. Keeps the GUI
                responsive so STOP can be clicked, and forwards the abort
                to the firmware the moment it is.
                """
                fig.canvas.flush_events()
                if state["abort"] and not state.get("abort_sent"):
                    state["abort_sent"] = True
                    print("STOP pressed - aborting path")
                    robot.abort()

            state["abort_sent"] = False
            ok = robot.queue_run(poll=poll)
            sync()                       # read back where it actually stopped

            if state["abort"]:
                print("path aborted. Position is still tracked, so no "
                      "re-home is needed.\n")
                return
            if not ok:
                return

            angles[:] = list(path[-1][1])
            redraw()
            robot.move_angles(*[centre[i] + offsets[i] for i in range(3)])
            sync()
            print("sweep complete. If the effector stayed level, "
                  "try a larger radius.\n")
        finally:
            state["busy"] = False

    # ---- buttons -------------------------------------------------------
    STEPS = [-10, -5, -1, 1, 5, 10]
    handles = []

    def make_jog(axis, delta):
        def handler(_event):
            if state["busy"]:
                return
            if robot.jog(axis, delta):
                sync()
            else:
                print("move rejected")
        return handler

    # Layout is laid out from the top down with explicit row positions so
    # adding a row cannot silently push the last one off the figure.
    h, w, gap = 0.038, 0.085, 0.006
    ROW = 0.045                       # vertical pitch between button rows
    x0 = 0.09

    def row_y(n):
        return 0.395 - n * ROW

    for n, axis in enumerate([0, 1, 2]):
        y = row_y(n)
        fig.text(0.03, y + h / 2, f"T{axis+1}", va="center", fontsize=11)
        for col, d in enumerate(STEPS):
            b = Button(plt.axes([x0 + col * (w + gap), y, w, h]), f"{d:+g}")
            b.on_clicked(make_jog(axis, d))
            handles.append(b)

    y = row_y(3) - 0.012
    fig.text(0.03, y + h / 2, "ALL", va="center", fontsize=11)
    for col, d in enumerate(STEPS):
        b = Button(plt.axes([x0 + col * (w + gap), y, w, h]), f"{d:+g}")
        b.on_clicked(make_jog(None, d))
        handles.append(b)

    # sweep row: circles at increasing radius, then the spiral
    y -= ROW + 0.012
    fig.text(0.03, y + h / 2, "SWEEP", va="center", fontsize=11)
    for col, r in enumerate([30, 50, 80, 100, 120]):
        b = Button(plt.axes([x0 + col * (w + gap), y, w, h]), f"R{r}")
        b.on_clicked(lambda _e, rr=r: run_sweep(float(rr)))
        handles.append(b)

    # spiral row: flat sweeps the plane, up/down also exercise Z
    y -= ROW + 0.012
    fig.text(0.03, y + h / 2, "SPIRAL", va="center", fontsize=11)
    b_sp_flat = Button(plt.axes([x0, y, w * 2 + gap, h]), "FLAT R80")
    b_sp_flat.on_clicked(lambda _e: run_spiral(args.spiral_radius))
    handles.append(b_sp_flat)

    b_sp_up = Button(plt.axes([x0 + 2 * (w + gap), y, w * 2 + gap, h]),
                     "UP R80")
    b_sp_up.on_clicked(
        lambda _e: run_cone(args.spiral_radius, abs(args.spiral_rise)))
    handles.append(b_sp_up)

    b_sp_dn = Button(plt.axes([x0 + 4 * (w + gap), y, w * 2 + gap, h]),
                     "DOWN R80")
    b_sp_dn.on_clicked(
        lambda _e: run_cone(args.spiral_radius, -abs(args.spiral_rise)))
    handles.append(b_sp_dn)

    # control row
    y -= ROW + 0.012
    b_home = Button(plt.axes([x0, y, 0.135, h]), "HOME (G28)")
    b_rel = Button(plt.axes([x0 + 0.145, y, 0.135, h]), "RELEASE (M0)")
    b_sync = Button(plt.axes([x0 + 0.290, y, 0.110, h]), "REFRESH")
    b_ws = Button(plt.axes([x0 + 0.410, y, 0.130, h]), "WORKSPACE")
    b_stop = Button(plt.axes([x0 + 0.550, y, 0.100, h]), "STOP")
    b_stop.on_clicked(lambda _e: state.__setitem__("abort", True))
    handles.append(b_stop)

    def on_workspace(_e):
        rows, best = report_workspace(args.angle_min, args.angle_max)
        if not best:
            return
        th = np.linspace(0, 2 * np.pi, 80)
        disc.set_data(best["usable_r"] * np.cos(th),
                      best["usable_r"] * np.sin(th))
        disc.set_3d_properties(np.full_like(th, best["z"]))
        print(f"  orange circle drawn at Z = {best['z']:.0f} mm\n")
        fig.canvas.draw_idle()

    def on_home(_e):
        if state["busy"]:
            return
        if robot.home():
            sync()
        else:
            print("homing failed")

    b_home.on_clicked(on_home)
    b_rel.on_clicked(lambda _e: robot.release())
    b_sync.on_clicked(lambda _e: sync())
    b_ws.on_clicked(on_workspace)
    handles += [b_home, b_rel, b_sync, b_ws]

    redraw()
    try:
        plt.show()
    finally:
        robot.close()


if __name__ == "__main__":
    main()
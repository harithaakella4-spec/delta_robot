# Vision-Guided Delta Robot

A computer-vision-guided pick-and-place delta robot, built as a proof of
concept for CV-driven industrial automation.

The robot sees objects on a flat surface, works out where each one is in
its own coordinate frame, and moves to them. Control firmware, kinematics,
calibration tooling and the vision pipeline are in this repository.

<p align="center">
  <img src="docs/robot.jpg" alt="The delta robot: three printed gearbox housings on a 2020 aluminium extrusion frame, printed upper arms, carbon fibre lower arms with rod ends, and the moving platform below" width="420">
</p>

<p align="center">
  <em>Current build. Printed gearbox housings on 2020 extrusion, carbon
  fibre lower arms with rod ends, moving platform below. No end effector
  fitted yet.</em>
</p>

**Status: partially working.** Motion, homing, camera calibration and
object detection all work independently. The complete pick-and-place cycle
does not yet run end to end — there is no gripper, and the drivetrain
limits the usable workspace. Known issues are listed below rather than
glossed over.

---

## Credit and provenance

The mechanical design and the reference firmware come from
**[Delta X](https://github.com/deltaxrobot)**, an open-source rotary delta
robot project. This is not an original mechanical design.

What was taken:

- The mechanical design and STL files, with **minor modifications** —
  small dimensional changes and print-orientation adjustments for a
  different printer and different sourcing. The geometry is unchanged.
- `Delta-X-Firmware` as a reference implementation, particularly its
  inverse kinematics, which was used to verify this code's own solver.

What was written here:

- Control firmware (`delta_homing.ino`) — the reference firmware did not
  run correctly on this hardware, so homing, jogging, path queueing and
  motion profiling were written from scratch
- Forward and inverse kinematics in Python, independently derived and
  cross-checked against the Delta X implementation
- Camera calibration, object detection and hand-eye calibration tooling
- The digital twin

Development was AI-assisted.

> **Licence note:** check the Delta X project's licence terms before
> redistributing anything derived from their design or firmware. That has
> not been resolved here yet.

---

## Hardware

| Component | Detail |
| --- | --- |
| Controller | Arduino Mega 2560 |
| Driver board | RAMPS 1.4 |
| Stepper drivers | 3 × A4988, 1/16 microstepping (all three jumpers fitted), Vref 0.7 V |
| Motors | 3 × NEMA 17 |
| Reduction | GT2 belt, 20T motor pulley to 60T output — 3:1 |
| Lower arms | Carbon fibre tube with rod ends |
| Endstops | Mechanical, pins 3 / 14 / 18, NO contacts wired to S and − |
| Power | 12 V |
| Camera | USB webcam, fixed overhead |

### Derived constants

```
STEPS_PER_DEG = (200 × 16 × 3) / 360 = 26.6667
INVERT        = { true, true, true }
```

Pulley centre distance measured from the CAD at **40.372 mm** across all
three arms (agreeing to 0.007 mm), giving a required belt length of
**160.7 mm** — nearest standard size is GT2 **80T, 160 mm, 6 mm wide**.

---

## Camera setup

Currently a USB webcam on a fixed mount looking down at the work surface.
The present rig is temporary and oblique, which costs accuracy: resolution
varies across the field and parallax error is at its worst.

**Intended setup:** centred under the base plate, looking straight down,
roughly 300–400 mm above the work plane.

- Centred minimises parallax, which is zero directly under the lens and
  grows toward the edges
- Higher means rays closer to vertical and a wider field of view, at the
  cost of pixels per millimetre — with calibration accurate to ~0.1 mm
  there is resolution to spare, so bias toward height
- The effector occludes the centre of the view while it works, which does
  not matter: vision is **look-then-move**, not continuous tracking. One
  clean frame is captured with the arms parked, then picks execute from
  that snapshot

### The height trap

A homography maps **one plane**. An object with height presents its top
face to the camera, displaced radially outward from the image centre, so
the detected centroid is not above where the object actually rests. The
error is zero directly under the lens and grows toward the edges.

Mitigations: mount high, calibrate at the height objects will be gripped
at rather than at the bare surface, or detect the base contour rather than
the whole blob.

---

## Software

| File | Purpose |
| --- | --- |
| `delta_homing.ino` | Firmware: homing, jogging, path queue, abort, endstop diagnostics |
| `twin_jog.py` | Joint-space jog GUI, live digital twin, workspace analysis, circle and spiral sweeps |
| `camera_test.py` | Camera preview and snapshot |
| `calibrate.py` | Camera → millimetre calibration on a measured reference |
| `detect.py` | Shape-agnostic object detection, outputs millimetre coordinates |
| `hand_eye.py` | Camera → **robot frame** calibration, using the robot as the measuring instrument |

### Firmware commands

Serial, 115200 baud, newline terminated.

```
G28                       home all three axes
H1 / H2 / H3              home one axis
D1 / D2 / D3              endstop noise diagnostic
J1 <deg> / JA <deg>       relative jog, one axis or all
M1 A<deg> B<deg> C<deg>   absolute move
Q A<deg> B<deg> C<deg>    queue a path point
X                         execute the queue as one continuous motion
!                         abort a move or path in flight
QC / QN                   clear queue / report depth
M92 S<steps_per_deg>      change scale at runtime
?                         status
M0 / M17                  drivers off / on
```

### Notable implementation details

**Two-stage homing.** A fast approach finds the switch, the arm retreats,
then creeps back slowly. The slow re-approach is what sets the reference.
A single fast approach stops at a slightly different angle each run
depending on momentum and lever flex, and on a delta that error
propagates directly into effector position.

**Path queueing.** Points are sent with `Q` and executed with `X` as one
continuous motion, with a single acceleration ramp spanning the whole
path. Sent individually, each point would be its own
accelerate-decelerate cycle and a swept circle visibly stutters.

**Mid-path abort.** `!` is checked every 64 steps during execution. The
machine stops where it is, holds position with the drivers energised, and
keeps counting steps — so position stays known and no re-home is needed.

**Validation before motion.** Every path point is checked against the
reachable workspace and the angle limits before anything moves. An
out-of-reach path is refused rather than discovered by collision.

**Endstop debouncing.** Readings require consecutive confirmations spread
over time, with the confirming reads taken while stepping is paused so the
drivers are quiet. Stepper switching couples into endstop wiring and a
single false reading makes homing believe a switch has fired.

---

## Measured figures

| Measurement | Value | How |
| --- | --- | --- |
| Homing repeatability | Better than one microstep (0.0375° at the shoulder, ≈0.05–0.12 mm at the tool) | Repeated `G28` from a fixed jog position |
| Camera calibration | ≈0.1 mm | Clicking known points against a measured reference |
| Verified working radius | 80 mm | Circle and spiral sweeps |
| Kinematics | FK/IK round-trip exact to 1e-14 | Numerical check |
| End-to-end accuracy | **Not yet measured** | Needs hand-eye calibration and a gripper |

---

## Known issues

**Belt slip above R80.** The belts were joined by hand with glue rather
than being closed loops, and the design's belt tensioners were not fitted.
Slip occurs at a fixed point in the belt's travel, which identifies the
splice rather than tension as the failing element. Fix is three GT2 80T
160 mm closed-loop belts plus the tensioner assembly (688ZZ bearings,
⌀9 × 16 mm springs, printed parts). Until then the working radius is
capped at 80 mm.

**Slight effector tilt.** The lower arms are not perfectly matched in
length. A delta's kinematics assume a rigid parallelogram, so a mismatch
tilts the platform and no software calibration corrects it. Currently
small enough to defer.

**No gripper.** A vacuum end effector is specified — 12 V diaphragm pump,
3-way solenoid valve, 15 mm bellows cup, 4 mm tubing — with the pump and
valve mounted on the frame rather than the effector to keep moving mass
low. Not yet built.

**Angle limits unverified.** `ANGLE_MIN` and `ANGLE_MAX` have not been
measured against the mechanism with the full linkage attached.

---

## A note on debugging

Worth recording because it cost two days and the lesson generalises.

Every commanded move overshot wildly and drove the arms into the motor
housings. Angle limits, home positions and speeds were all adjusted in
turn, and the reference firmware was tried and failed too. The cause was a
single constant: `STEPS_PER_DEG` had been taken from the reference
project's config, which assumes 1/16 microstepping. The board had **no
microstepping jumpers fitted at all** and was running at full step —
sixteen times too many steps per command.

What exposed it was building the digital twin. Every number in the system
was internally consistent and wrong; putting a rendered model beside the
physical machine made the disagreement obvious in seconds.

When a system is consistently wrong, the fault is usually in an assumption
nobody tested. Building something that makes the assumption visible beats
adjusting everything downstream of it.

---

## Roadmap

- [ ] Replace glued belts with closed-loop GT2, fit tensioners
- [ ] Match lower arm lengths to remove effector tilt
- [ ] Measure real mechanical angle limits
- [ ] Build and integrate the vacuum gripper
- [ ] Hand-eye calibration against the robot frame
- [ ] Close the loop: detect → plan → pick → place
- [ ] Measure end-to-end accuracy and sustained reliability
- [ ] Permanent overhead camera mount

---

## Running it

```bash
pip install pyserial matplotlib numpy opencv-python

# flash delta_homing.ino to the Mega first, then:
python twin_jog.py --mode sim                      # no hardware needed
python twin_jog.py --mode arduino --port COM6      # live

python camera_test.py --device 1                   # find the camera
python calibrate.py --rect 297 210 --device 1      # camera -> mm
python detect.py --device 1                        # detection
python hand_eye.py --port COM6 --device 1          # camera -> robot
```

Close the Arduino IDE's Serial Monitor before running anything in Python —
only one program can hold the port.
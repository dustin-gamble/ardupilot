# Agent Handoff — Spin-Wing VTOL

Pick-up doc for continuing this work on another machine, or by a future agent.

**Project:** fork of ArduPilot `Plane-4.5` adding a custom VTOL flying wing — a
twin-pusher tailsitter with a wing-tilt servo acting as a helicopter-style
collective in hover, plus a non-spinning virtual-heading HUD, plus a
once-per-revolution cyclic mixer on the elevons that compensates for
gyroscopic precession so the autopilot can hold position on a deliberately
spinning airframe.

**Status:** running in SITL + RealFlight FlightAxis + Mission Planner.
Manual heli-style control in QSTABILIZE works (RC10 switch → motors,
throttle stick → wing-tilt collective). Autopilot collective + cyclic in
QHOVER/QLOITER are coded and live but **not yet flight-verified**.

Branch: `spinwing-vtol` on https://github.com/dustin-gamble/ardupilot.

---

## What's in the branch

Three implementation phases on top of `upstream/Plane-4.5`:

### Phase 1 — wing-tilt servo + virtual-heading HUD
- `SRV_Channel::k_wing_tilt_collective` = enum value **156** (next free in Plane-4.5)
- `WING_CRUISE_PWM` / `WING_HOVER_PWM` / `VHDG_ENABLE` params
- `QuadPlane::output_wing_tilt()` drives SERVO5 each loop
- `QuadPlane::get_virtual_heading_rad()` returns commanded-velocity direction
- `send_attitude()` swaps ATTITUDE.yaw for virtual heading in VTOL modes
- `BYAW` (real body yaw) + `SPN_RPM` (spin rate) `NAMED_VALUE_FLOAT` diagnostics

### Phase 2 — analog collective
- `output_wing_tilt()` now scales SERVO5 PWM **with the autopilot throttle**
  between `WING_CRUISE_PWM` and `WING_HOVER_PWM`. Replaces the original
  binary cruise/hover output. Wing is a true helicopter collective.

### Phase 3 — spin-cyclic mixer (gyroscopic precession compensated)
- `QuadPlane::output_spin_cyclic()` hooked from `Tailsitter::output()`
  after the standard elevon mix. When body yaw rate > `SPIN_THRSHLD`,
  overrides body-frame elevon outputs with a **once-per-revolution sine
  wave** that produces a steady world-frame tilt:
  `cyclic = cos(ahrs_view.yaw + SPIN_PH_LEAD - tilt_dir)`
- Tilt-demand source depends on mode:
  - QSTABILIZE/QACRO: RC pitch+roll sticks rotated into virtual-heading
    frame (forward stick = "go toward HUD compass direction")
  - QHOVER/QLOITER/QLAND/QRTL/QAUTO: `pos_control->get_accel_target_cmss()`
    (standard pos controller, autopilot owns position)
- New params: `SPIN_ENABLE`, `SPIN_THRSHLD`, `SPIN_PH_LEAD`, `SPIN_CYC_GAIN`
- New telemetry: `SPN_CYC` (cyclic output -1..+1), `SPN_PHS` (body azimuth
  in deg, sweeps -180..+180 per rev), and **ESC3 RPM** carries spin rate
  via `AP::esc_telem().update_rpm(2, rpm, 0)` for the MP HUD gauge.

### Heli-style manual control (bypasses AP_MotorsTailsitter spool state)
The standard `AP_MotorsTailsitter` mixer stays in `SHUT_DOWN` spool state
on this branch for reasons we never fully diagnosed — motor PWM stuck at
`Q_M_PWM_MIN` regardless of throttle. To work around it:
- `output_spin_manual_motors()` drives `k_throttleLeft/Right` (functions
  73/74, *not* `k_motor1/2` = 33/34) directly from a 3-pos RC switch
  with PWM presets `SPIN_THR_LO/MID/HI`. Active in ALL VTOL modes so
  the pilot always owns motor RPM / spin rate.
- Hook is in `Tailsitter::output()` AFTER `Plane::servos_twin_engine_mix()`
  — otherwise the twin-engine mixer overwrites our PWM with `k_throttle`
  (which is 0 in VTOL).
- In QSTABILIZE/QACRO `output_wing_tilt()` reads RC3 (`SPIN_COL_RCCH`)
  directly as the pilot's analog collective stick — manual collective.
- In QHOVER/QLOITER `output_wing_tilt()` reads `motors->get_throttle()`
  (autopilot), clamped to `[SPIN_COLL_MIN, WING_HOVER_PWM]` so the
  controller can only modulate a narrow band around hover (won't collapse
  the wing to cruise = no lift).
- New params: `SPIN_MAN_ENBL`, `SPIN_MOT_RCCH`, `SPIN_COL_RCCH`,
  `SPIN_THR_LO/MID/HI`, `SPIN_COLL_MIN`.

### Tailsitter axis bug fixes
Important: for a tailsitter at nose-up hover, the airframe spin axis is
**body X** (the nose), NOT body Z (which becomes horizontal). Multiple
places in our code originally read raw body Z — those were reading the
wobble axis, not the spin axis. Fixed:
- `SPN_RPM` now uses `get_spin_rate_rps()` which reads `-ahrs_view.z`
- `output_spin_cyclic()` phase now uses `ahrs_view->yaw` (not `ahrs.get_yaw()`)
- `get_spin_phase_rad()` is the public accessor

The AHRS view is rotated 90 deg pitch (see `QuadPlane::setup()` line 776)
so view-frame Z = world DOWN and view-frame yaw = rotation about world
vertical = the actual spin angle.

---

## Get going on a new machine

### 1. Clone + branch
```bash
git clone https://github.com/dustin-gamble/ardupilot.git ardupilot-spinwing
cd ardupilot-spinwing
git checkout spinwing-vtol
git submodule update --init --recursive
```

### 2. Add the upstream remote (optional, for future rebases)
```bash
git remote add upstream https://github.com/ArduPilot/ardupilot.git
git fetch upstream Plane-4.5
```

### 3. Build environment
**Linux / WSL2 Ubuntu (recommended for SITL):**
The official `Tools/environment_install/install-prereqs-ubuntu.sh -y` works
but installs ~1 GB of stuff. For SITL only, the lean path is:
```bash
pip3 install --user empy==3.3.4 pexpect future dronecan MAVProxy "setuptools<81"
ln -sf /usr/bin/python3 ~/.local/bin/python   # waf-light shebang wants `python`
```
Plus apt: `g++`, `gawk`, `make`, `libtool`, `libxml2-dev` (usually already
installed on Ubuntu 22.04 base). No sudo needed beyond that.

**macOS:** Homebrew Python is externally-managed, use a venv. See the
project README of the original session for details.

### 4. Build SITL
```bash
export PATH="$HOME/.local/bin:$PATH"
./waf configure --board sitl
./waf plane
```
Produces `build/sitl/bin/arduplane` (~5 MB). Should finish clean.

CubeOrange target NOT verified — needs `arm-none-eabi-gcc` on a hardware
test machine. All changes are board-agnostic or guarded by
`HAL_QUADPLANE_ENABLED`, which is on for CubeOrange.

### 5. Run SITL + RealFlight FlightAxis
```bash
source venv/bin/activate   # if macOS
sim_vehicle.py -v ArduPlane -f flightaxis:<RF_HOST_IP> \
    --add-param-file=spinwing.parm \
    --add-param-file=spinwing-first-flight.parm \
    --out=udpout:<MP_HOST_IP>:14550 \
    --no-extra-ports \
    --mavproxy-args="--daemon"
```
For Windows+WSL2 the helper scripts in `sitl-launch/` (one directory up
from the cloned repo) wrap this — see `sitl-launch/run-sitl-spinwing.sh`.

---

## RealFlight model channel map (required)

The `.RFX` aircraft can't be authored from source — build it in RealFlight
Editor as a **twin-pusher tailsitter flying wing** with a variable-incidence
wing mechanism, spawning **nose-up vertical** on its tail. Channel map:

| Channel | Function | `SERVOn_FUNCTION` |
|---------|----------|-------------------|
| 1 | Left pusher motor  | **73** (ThrottleLeft) |
| 2 | Right pusher motor | **74** (ThrottleRight) |
| 3 | Left elevon  | 77 (ElevonLeft) |
| 4 | Right elevon | 78 (ElevonRight) |
| 5 | Wing-tilt collective | **156** (k_wing_tilt_collective, our custom enum) |

**Function-number gotcha:** `AP_MotorsTailsitter` writes motor PWM to
`k_throttleLeft`/`k_throttleRight` = **73/74**, NOT `k_motor1`/`k_motor2`
= 33/34. Using 33/34 with `Q_FRAME_CLASS=10` gives zero motor output.

Important physical requirements for stable spin:
- **Both motors spin the same direction** (both CW from pilot view, or
  both CCW). Counter-rotating props kill the torque that drives spin.
- **Prop thrust line passes through CG.** Offset thrust precesses the
  spin axis = wobble = EKF struggles.
- **Spin rate target: 60-120 RPM** (1-2 Hz). Below ~30 RPM the cyclic
  mixer is gated off; above ~150 RPM the EKF stops being reliable.

---

## Heli-style flight model

Once the model is set up and flying:

| Stick / switch | Function in QSTABILIZE | Function in QLOITER |
|---|---|---|
| RC10 (3-pos switch) | Motor PWM preset (`SPIN_THR_LO/MID/HI`) | Same |
| RC3 (throttle stick) | Wing-tilt collective (1000..2000 PWM) | (autopilot owns altitude) |
| RC1 (roll) + RC2 (pitch) | Cyclic stick → world-frame tilt demand (virtual-heading frame) | (autopilot owns position) |
| Mode switch | QSTABILIZE (pos 1-4) → QHOVER (pos 5) → QLOITER (pos 6) | |

Test sequence:
1. Reset RealFlight (Spacebar)
2. Arm (MP Actions tab or stick combo)
3. RC10 → MID (motors at idle PWM, e.g. 1650)
4. RC10 → HIGH (motors at flight PWM, e.g. 2000) — body should start spinning
5. Push throttle stick up → SERVO5 PWM rises → wing AoA up → vertical lift
6. With aircraft hovering, push cyclic stick → body drifts in commanded direction
7. Switch to QHOVER for altitude hold, then QLOITER for position hold

---

## Tuning quickstart

| Symptom | Param to adjust |
|---|---|
| Motion drifts at 45-90° offset from stick direction | `SPIN_PH_LEAD` ± 45° from 90° |
| Aircraft oscillates around target | `SPIN_CYC_GAIN` lower (15, 10) |
| Aircraft sluggish, doesn't respond to stick | `SPIN_CYC_GAIN` higher (35, 50) |
| Autopilot swings collective wildly in QLOITER | `SPIN_COLL_MIN` higher (1750, 1800) |
| Can't take off in QLOITER (wing too low AoA) | `SPIN_COLL_MIN` to 1850+ or `WING_HOVER_PWM` to 2100 |
| Spin too fast | `SPIN_THR_HI` lower (1700, 1500) |
| HUD attitude wobbles even with VHDG | model is precessing; check prop-thrust-through-CG |
| EKF lane switches mid-flight | check `EK3_IMU_MASK=1` is set (single IMU) |

All params can be edited live via MP Full Parameter List + Write Params
(no reboot needed unless marked `@RebootRequired`).

---

## Mission Planner parameter descriptions

All 14 new SPIN_*/WING_*/VHDG_ENABLE params have full `@Param`/`@Description`/
`@Range`/`@Units` metadata in `Parameters.cpp`. To install them into MP so
the Full Parameter List shows the descriptions:

1. Run `python3 Tools/autotest/param_metadata/param_parse.py --vehicle ArduPlane --format xml`
   from this repo root. Produces `apm.pdef.xml`.
2. Extract the SPIN_/WING_/VHDG_ entries from `apm.pdef.xml` and inject
   them into `C:\Program Files (x86)\Mission Planner\ParameterMetaDataBackup.xml`
   under the `<ArduPlane>` section. The `sitl-launch/install-mp-param-meta.ps1`
   script automates this with idempotent re-runs and an auto-backup.
3. Restart Mission Planner.

---

## Open items / next session

1. **`SPIN_PH_LEAD` empirical tuning** — default 90°, user reported motion
   was 45° off; try 45° or 135° in MP.
2. **QLOITER position hold flight test** — with new `SPIN_COLL_MIN=1700`
   clamp the wing should stay near hover and the autopilot should hold
   position via cyclic. Not yet verified.
3. **AP_MotorsTailsitter SHUT_DOWN bypass** — we work around the spool
   state lock by writing PWM directly. Root cause unknown — worth
   diagnosing for upstream-friendly fix.
4. **Body precession / wobble** at high spin rate — partially mitigated by
   EK3 tuning, but ideally fixed in the RealFlight model (prop thrust
   through CG, balanced wing inertia).
5. **CubeOrange hardware build** — never verified; the design is
   board-agnostic but ARM compile needs an ARM toolchain machine.
6. **`.RFX` model** — user has a working one in RealFlight; not in repo
   since it's binary and not source-controlled.

---

## Repository layout

```
ardupilot-spinwing/
├── ArduPlane/
│   ├── Parameters.{h,cpp}   # 14 new SPIN_/WING_/VHDG_ params
│   ├── quadplane.{h,cpp}    # output_wing_tilt, output_spin_cyclic,
│   │                        # output_spin_manual_motors,
│   │                        # get_spin_rate_rps, get_spin_phase_rad,
│   │                        # get_virtual_heading_rad
│   ├── tailsitter.cpp       # hooks output_spin_cyclic + output_spin_manual_motors
│   │                        # AFTER the standard elevon mix
│   └── GCS_Mavlink.cpp      # BYAW, SPN_RPM, SPN_CYC, SPN_PHS NAMED_VALUE_FLOATs
│                            # + ATTITUDE yaw override + ESC3 RPM telem
├── libraries/SRV_Channel/SRV_Channel.h   # k_wing_tilt_collective = 156
├── spinwing.parm            # SITL defaults: tailsitter setup + EKF tolerances
├── BUILD_SPINWING.md        # Build + run guide (still valid)
├── HANDOFF.md               # This file
└── AGENT.md                 # Instructions for future Claude/agent sessions
```

For a single-machine Windows+WSL2 setup, the parent dir of the repo also
contains `sitl-launch/` with launcher / probe / sync scripts.

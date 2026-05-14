# Spin-Wing VTOL — Build & Run (SITL + FlightAxis)

Fork of ArduPilot's `Plane-4.5` adding a custom VTOL flying wing: a twin-pusher
tailsitter with a wing-tilt servo that acts as a helicopter-style collective in
hover, and a non-spinning virtual-heading HUD.

Branch: `spinwing-vtol` (off `upstream/Plane-4.5`).

## What changed

| # | Commit | Change |
|---|--------|--------|
| 1 | `SRV_Channel: add k_wing_tilt_collective servo function` | New `SERVO_FUNCTION` enum value **156** (`k_wing_tilt_collective`) + GCS `@Values` metadata. |
| 2 | `ArduPlane: drive wing-tilt servo from QuadPlane update loop` | `QuadPlane::output_wing_tilt()`, called every loop from `QuadPlane::update()`. Phase 1 binary cruise/hover; phase 2 analog collective is a commented one-liner toggle. |
| 3 | `ArduPlane: add QuadPlane virtual heading computation` | `QuadPlane::get_virtual_heading_rad()` — reports commanded-velocity direction, holds last heading when stationary. |
| 4 | `ArduPlane: override ATTITUDE yaw with virtual heading in VTOL` | `send_attitude()` swaps the yaw field for the virtual heading in VTOL modes; raw gyro Z untouched. Publishes `BYAW` + `SPN_RPM` `NAMED_VALUE_FLOAT` diagnostics. |
| 5 | `ArduPlane: add spin-wing VTOL parameters` | `WING_CRUISE_PWM`, `WING_HOVER_PWM`, `VHDG_ENABLE` in the `Parameters` (g) block. |
| 6 | `spinwing: add SITL parameter file` | `spinwing.parm` — EKF3 tolerances, tailsitter setup, servo map. |

All ArduPlane/QuadPlane code is inside `#if HAL_QUADPLANE_ENABLED`.

### Deviations from the original design sketch

- **`SERVO_FUNCTION` value is 156, not 187.** On `Plane-4.5` the enum ends at
  `k_rcin16_mapped = 155`; 156 is the next free value. `spinwing.parm` uses
  `SERVO5_FUNCTION 156` to match.
- **`Q_TAILSIT_ENABLE 1` added to `spinwing.parm`.** It was missing from the
  sketch; `tailsitter.enabled()` is `(enable > 0) && setup_complete`, so the
  tailsitter code path never activates without it.
- **`get_virtual_heading_rad()` uses `pos_control->get_vel_target_cms()`** —
  `get_vel_target_NEU_cms()` does not exist on `Plane-4.5`'s `AC_PosControl`.
- **`BYAW`/`SPN_RPM` are emitted from `send_attitude()`** (per-channel, at the
  ATTITUDE stream rate) — ArduPlane has no pre-existing named-float periodic.

## Prerequisites

macOS with Homebrew Python is *externally managed* (PEP 668), so the ArduPilot
Python build deps go in a venv. From the repo parent directory:

```bash
python3 -m venv --system-site-packages venv-ardupilot
venv-ardupilot/bin/pip install empy==3.3.4 pexpect future dronecan "setuptools<81"
```

Notes:
- `empy==3.3.4` — exact version; ArduPilot's waf codegen requires it.
- `dronecan` — needed by the `dronecangen` build step.
- `setuptools<81` — setuptools 81+ removed `pkg_resources`, which `dronecan`
  imports.

`gh` (GitHub CLI) is used for the fork remote; install with `brew install gh`.

## Build (SITL)

```bash
source ../venv-ardupilot/bin/activate
./waf configure --board sitl
./waf plane
```

Output: `build/sitl/bin/arduplane`. Incremental rebuilds are a few seconds.

> CubeOrange target: not built here (ARM toolchain not installed in this
> environment). All changes are board-agnostic or guarded by
> `HAL_QUADPLANE_ENABLED`, which is enabled on CubeOrange — verify with
> `./waf configure --board CubeOrange && ./waf plane` on a box with the
> `arm-none-eabi` toolchain.

## Run (SITL + FlightAxis / RealFlight)

```bash
source ../venv-ardupilot/bin/activate
sim_vehicle.py -v ArduPlane -f flightaxis --console --map \
    --add-param-file=spinwing.parm
```

`flightaxis` bridges to RealFlight 9.5+ over the network — RealFlight must be
running with the aircraft loaded and "FlightAxis Link" enabled. For a quick
code-only check without RealFlight, substitute `-f quadplane` (no wing-tilt
physics, but the servo output, virtual heading, and named-float telemetry are
all exercisable).

## RealFlight model

A binary RealFlight aircraft (`.RFX`) can't be authored from source — build it
in RealFlight's editor as a **twin-pusher tailsitter flying wing** with a
variable-incidence wing mechanism, and match channel mapping to `spinwing.parm`:

| Channel | Function | `SERVOn_FUNCTION` |
|---------|----------|-------------------|
| 1 | Left pusher motor  | 33 (ThrottleLeft) |
| 2 | Right pusher motor | 34 (ThrottleRight) |
| 3 | Left elevon  | 77 (ElevonLeft) |
| 4 | Right elevon | 78 (ElevonRight) |
| 5 | Wing-tilt collective | 156 (k_wing_tilt_collective) |

The wing-tilt servo should drive wing incidence between the cruise position
(`WING_CRUISE_PWM`, default 1000) and the high-incidence hover position
(`WING_HOVER_PWM`, default 2000).

## Test plan

1. **Servo wiring** — boot, watch SERVO5 PWM; toggle FBWA ↔ QSTABILIZE and
   confirm it moves between `WING_CRUISE_PWM` and `WING_HOVER_PWM`.
2. **Virtual heading** — in QHOVER with the body spinning, the HUD compass
   (ATTITUDE yaw) should stay near-constant; `BYAW` `NAMED_VALUE_FLOAT` shows
   the real, spinning body yaw.
3. **Spin RPM** — `SPN_RPM` `NAMED_VALUE_FLOAT` shows live spin rate (rev/min).
4. **Hover stability** — tune `Q_A_RAT_*`; use QAUTOTUNE if available.
5. **Full transition** — QSTABILIZE takeoff → FBWA cruise → QLOITER → QLAND.
6. **Phase 2** — uncomment the analog-collective block in
   `QuadPlane::output_wing_tilt()`, rebuild, confirm throttle modulates wing
   tilt continuously in hover.

## Open items

- `Q_TAILSIT_MOTMX 5` is carried over from the design sketch — verify it
  against the actual motor matrix for `Q_FRAME_CLASS 10`; a 2-motor setup may
  want `3` (motors 1+2).
- `Q_A_RAT_YAW_*` may need retuning or disabling in spin mode — body yaw is
  uncontrolled by design.
- The virtual-heading override changes ATTITUDE yaw globally; Mission Planner
  uses that field for some navigation displays, so the override may eventually
  need to be HUD-selective.

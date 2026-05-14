# Agent Handoff — Spin-Wing VTOL

Pick-up doc for continuing this work on another machine.

**Project:** fork of ArduPilot `Plane-4.5` adding a custom VTOL flying wing — a
twin-pusher tailsitter with a wing-tilt servo acting as a helicopter-style
collective in hover, plus a non-spinning virtual-heading HUD.

**Status:** all 6 planned code changes are **done, committed (one per change),
built clean for SITL, and pushed**. Branch `spinwing-vtol` on
`https://github.com/dustin-gamble/ardupilot`. Remaining work is verification,
not implementation — see "What's left" below.

---

## Get going on a new machine

### 1. Clone the fork and check out the branch
```bash
git clone https://github.com/dustin-gamble/ardupilot.git ardupilot-spinwing
cd ardupilot-spinwing
git checkout spinwing-vtol
git submodule update --init --recursive
```

### 2. Add the upstream remote
The clone gives you `origin` = the fork. Add ArduPilot as `upstream`:
```bash
git remote add upstream https://github.com/ArduPilot/ardupilot.git
git fetch upstream Plane-4.5
```

### 3. Build environment — the non-obvious part
ArduPilot's SITL build runs Python codegen (`empy`, `dronecangen`). On macOS,
Homebrew Python is externally-managed (PEP 668), so global `pip install` is
blocked. Use a venv:
```bash
python3 -m venv --system-site-packages ../venv-ardupilot
../venv-ardupilot/bin/pip install empy==3.3.4 pexpect future dronecan "setuptools<81"
```
Why these:
- `empy==3.3.4` — exact version; waf codegen requires it.
- `dronecan` — needed by the `dronecangen` build step.
- `setuptools<81` — setuptools 81+ removed `pkg_resources`, which `dronecan` imports.

On Linux, ArduPilot's `Tools/environment_install/install-prereqs-ubuntu.sh -y`
handles all of this and you can skip the venv.

### 4. Build SITL
```bash
source ../venv-ardupilot/bin/activate   # macOS venv; skip on Linux
./waf configure --board sitl
./waf plane
```
Produces `build/sitl/bin/arduplane`. Should finish clean — if it doesn't, the
code is known-good as of commit `99dfb73`, so suspect the environment.

### 5. gh CLI (only needed to push to the fork)
```bash
brew install gh && gh auth login    # GitHub.com -> HTTPS -> web browser
```

---

## What's done

Branch `spinwing-vtol`, 7 commits on top of `upstream/Plane-4.5` (`5ddc3da537`):

| # | Commit | Change |
|---|--------|--------|
| 1 | `84fd9d0` | `k_wing_tilt_collective` SERVO_FUNCTION enum = **156** + GCS `@Values` metadata |
| 5 | `96d5671` | `WING_CRUISE_PWM` / `WING_HOVER_PWM` / `VHDG_ENABLE` params (`Parameters` g block) |
| 2 | `f70199e` | `QuadPlane::output_wing_tilt()`, called each loop from `QuadPlane::update()` |
| 3 | `f0ed444` | `QuadPlane::get_virtual_heading_rad()` |
| 4 | `7c003eb` | ATTITUDE yaw override in VTOL modes + `BYAW`/`SPN_RPM` `NAMED_VALUE_FLOAT` |
| 6 | `f2b44b1` | `spinwing.parm` SITL parameter file |
| — | `99dfb73` | `BUILD_SPINWING.md` build/run guide |

These are committed and pushed. **Do not re-implement them.** Full detail,
test plan, and where the implementation deviates from the original design
sketch are in `BUILD_SPINWING.md`.

## What's left (verification + deliverables, not implementation)

1. **CubeOrange build check.** The plan requires clean compilation for `sitl`
   *and* `CubeOrange`. Only SITL was verified (the original machine had no ARM
   toolchain). On a machine with `arm-none-eabi-gcc`:
   ```bash
   ./waf configure --board CubeOrange && ./waf plane
   ```
   All code is board-agnostic or guarded by `HAL_QUADPLANE_ENABLED` (on for
   CubeOrange), so it should build — but it needs to be confirmed.
2. **Telemetry `.bin` log.** A deliverable: a dataflash log showing a stable
   hover with the virtual heading holding steady while the body spins. Needs a
   running SITL session — ideally `-f flightaxis` with RealFlight 9.5+, or
   `-f quadplane` for a physics-light smoke test. Run command and test plan are
   in `BUILD_SPINWING.md`.
3. **`realflight_model.RFX`.** A RealFlight aircraft file — binary, can't be
   authored from source. The required channel map is in `BUILD_SPINWING.md`;
   build the model in RealFlight's editor to match.

## Open items (phase 2+, from the design sketch)

- `Q_TAILSIT_MOTMX 5` in `spinwing.parm` is carried over from the sketch —
  verify against the actual motor matrix for `Q_FRAME_CLASS 10`; a 2-motor
  setup may want `3` (motors 1+2).
- Phase 2 analog collective: uncomment the throttle-scaled block in
  `QuadPlane::output_wing_tilt()` (`ArduPlane/quadplane.cpp`), rebuild, verify.
- `Q_A_RAT_YAW_*` may need retuning or disabling in spin mode — body yaw is
  uncontrolled by design.
- The virtual-heading override changes ATTITUDE yaw globally; Mission Planner
  uses that field for some navigation displays, so it may eventually need to be
  HUD-selective.

## Keeping the fork current with upstream

The original session ran a `/loop` that hourly checks `upstream/Plane-4.5` and
rebases if it moved. `Plane-4.5` is a slow-moving stable branch; as of the last
check it had not moved from `5ddc3da537`. To do this check manually:
```bash
git fetch upstream Plane-4.5
git rev-list --count spinwing-vtol..upstream/Plane-4.5   # 0 = nothing to rebase
# if non-zero:
git rebase upstream/Plane-4.5 && ./waf plane && git push --force-with-lease origin spinwing-vtol
```

## See also

- `BUILD_SPINWING.md` — full build/run guide, RealFlight channel map, test
  plan, and the list of deviations from the original design sketch.

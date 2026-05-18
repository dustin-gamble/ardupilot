# AGENT.md — instructions for future Claude/Cursor/Aider sessions

You are picking up the **Spin-Wing VTOL** project. Read [HANDOFF.md](HANDOFF.md)
first for the technical state. This document tells you how to *operate* on
this project, what conventions to follow, and what NOT to break.

---

## What this project is, in one paragraph

A custom ArduPlane fork that adds a **spinning-tailsitter** flight mode: a
twin-pusher flying wing whose airframe deliberately rotates about its yaw
axis in hover. A wing-tilt servo acts as a helicopter-style collective. The
elevons are driven by a once-per-revolution cyclic mixer that compensates
for gyroscopic precession so the autopilot can translate horizontally
despite the body spinning. The HUD compass is held steady via a
virtual-heading override so the GCS pilot isn't disoriented.

---

## Before you do anything

Run this diagnostic to understand the current state of the world:

```bash
# What branch and where:
cd ~/ardupilot-spinwing && git branch --show-current && git status --short && git log --oneline -5

# What's running:
pgrep -af 'arduplane|mavproxy|sim_vehicle'

# What the user's setup looks like:
cat ~/.claude/projects/C--Users-dusti-OneDrive-Documents-spin-wing/memory/MEMORY.md
```

If git status is dirty, **understand what's uncommitted before changing
anything else** — the prior session may have left work in progress.

---

## Repository topology (this is unusual — read carefully)

The user works on **Windows + WSL2 Ubuntu 22.04**. There are TWO working
copies of the repo:

| Path | Purpose | Edit method |
|---|---|---|
| `C:\Users\dusti\OneDrive\Documents\spin-wing\ardupilot\` | Windows clone, used for editing via the Edit/Write tool | Use `Edit`/`Write` tools at these paths |
| `~/ardupilot-spinwing/` in WSL | Build copy, builds via `./waf plane`, has the proper git remote | Modified via `sync-and-build.sh` which mirrors edits from Windows clone |

**Rule:** edit at `C:\Users\...\spin-wing\ardupilot\` (visible to MP and to
your tools), then run `sitl-launch/sync-and-build.sh` to mirror to WSL and
rebuild. Don't try to edit files at the `~/ardupilot-spinwing/` path
directly via WSL — the tool cross-shell quoting is brittle and the user's
editor of choice points at the Windows clone.

Helper scripts live one directory up at `C:\Users\dusti\OneDrive\Documents\spin-wing\sitl-launch\`.

---

## Critical gotchas that will bite you

### 1. PowerShell vs git-bash for invoking wsl

`/mnt/c/...` paths fail when wsl is invoked from git-bash because MSYS
rewrites them to `C:/Program Files/Git/mnt/c/...`. **Use the PowerShell
tool when invoking `wsl` with `/mnt/c/...` arguments.** Bash tool is OK
for everything else.

### 2. PowerShell quoting munges shell special chars

`|` inside a PowerShell-passed-bash command gets interpreted as a
PowerShell pipe. `\r` in `tr -d "\r"` gets stripped, deleting all the r's
in the file. **Workaround: always put non-trivial WSL commands in a
`.sh` script file in `sitl-launch/`, then invoke as
`wsl -d Ubuntu-22.04 -- bash /mnt/c/.../sitl-launch/script.sh`.**

### 3. Tailsitter axis swap

The AHRS view is rotated 90° pitch for tailsitters (see `QuadPlane::setup()`
line 776: `ROTATION_PITCH_90`). Consequences:
- The body's **spin axis** = body X (when nose-up) = view -Z
- Reading `ahrs.get_gyro().z` gives the *wobble* axis (body Z = horizontal)
- For spin rate, use `get_spin_rate_rps()` (already handles the swap)
- For spin azimuth angle, use `get_spin_phase_rad()` or `ahrs_view->yaw`

If you write any new code that needs "spin rate" or "rotation about
vertical", **use those accessors**, not raw `ahrs.get_gyro()` / `ahrs.get_yaw()`.

### 4. Tailsitter motor function numbers

`AP_MotorsTailsitter::output_to_motors()` writes to `k_throttleLeft` (73)
and `k_throttleRight` (74), NOT `k_motor1`/`k_motor2` (33/34). The 33/34
values are only written by `AP_MotorsMatrix` (multicopter). Setting
`SERVO1/2_FUNCTION = 33/34` with `Q_FRAME_CLASS = 10` produces ZERO motor
output (channels listen on functions nobody writes to).

### 5. Tailsitter::output() runs AFTER QuadPlane::update()

The main loop order:
1. `QuadPlane::update()` (in scheduler) — motor mixer, our `output_wing_tilt()`
2. `Plane::servos_output()` (in fast_loop) — calls `Tailsitter::output()` AND `servos_twin_engine_mix()`

Any write to `k_throttleLeft/Right` from `QuadPlane::update()` gets
*overwritten* by `servos_twin_engine_mix()` (which sets them based on
`k_throttle`, which is 0 in VTOL). To write motor PWM that sticks, hook
into `Tailsitter::output()` AFTER its existing writes. We do this for
`output_spin_manual_motors()` — see [tailsitter.cpp](ArduPlane/tailsitter.cpp)
line ~496.

### 6. AP_MotorsTailsitter spool state is stuck SHUT_DOWN

There's a long-standing bug we never diagnosed where the standard
QuadPlane motor mixer stays in `SHUT_DOWN` even when armed in QHOVER /
QLOITER with throttle stick up. Motors output `Q_M_PWM_MIN` (=1100)
regardless. The workaround is the heli-style direct-PWM path
(`output_spin_manual_motors`). **If a future task is "make standard
QuadPlane VTOL work," that's the root cause to find and fix.**

### 7. SITL launcher quirks

- `nohup` from background bash strips the `~/.local/bin` PATH that
  `waf-light`'s `#!/usr/bin/env python` shebang needs. The launcher
  script (`~/run-sitl-spinwing.sh`) exports PATH explicitly. **Don't
  remove that line.**
- MAVProxy will exit immediately if stdin is closed (background mode).
  Use `--mavproxy-args="--daemon"` — already in the launcher.
- The user typically has Mission Planner pre-bound to UDP 14550. MP's
  built-in SITL launcher will NOT work — it loads stock `arduplane.exe`
  which has none of our spin-wing code. The user must launch via our
  WSL launcher.

---

## When the user asks for changes

### Always

- Edit files in the Windows clone (`C:\Users\...\ardupilot\`)
- Run `sitl-launch/sync-and-build.sh` to mirror + rebuild
- Launch via `sitl-launch/run-sitl-spinwing.sh`
- Verify with the appropriate `probe-*.py` script (don't ask the user to
  read raw MAVLink — pick the relevant probe or write a new one)
- For new params: add full `@Param`/`@DisplayName`/`@Description`/
  `@Range`/`@Units` metadata in `Parameters.cpp` so MP descriptions stay
  complete. Then run `sitl-launch/install-mp-param-meta.ps1` to refresh
  the MP descriptions (idempotent — safe to re-run).

### Never

- **Never** commit `mav_*.parm` (MAVProxy runtime artifacts). Already in
  `.git/info/exclude`.
- **Never** use `git rebase -i` or `git push --force` without explicit
  user permission — this branch is shared and the prior agent's history
  matters.
- **Never** edit at the `~/ardupilot-spinwing/` path via WSL directly —
  it'll be overwritten by the next sync from Windows.
- **Never** introduce dependencies on `motors->armed()` returning true
  for spin-wing controls — the QuadPlane spool state bug means it often
  doesn't, even when armed. Use `plane.arming.is_armed_and_safety_off()`
  for arm checks.

---

## Common operations recipe book

### Add a new parameter

1. Add enum slot in `ArduPlane/Parameters.h` (in the spin-wing block)
2. Add member: `AP_Int8`/`AP_Int16`/`AP_Float` in `Parameters` class
3. Add `GSCALAR(...)` row in `Parameters.cpp` WITH full `@Param`/`@Description`/etc. metadata
4. Use as `plane.g.your_param` from anywhere
5. Sync, rebuild
6. Optional: re-run `sitl-launch/install-mp-param-meta.ps1` to install
   description into MP (after extracting the new entry from regenerated
   `apm.pdef.xml`)

### Add a new NAMED_VALUE_FLOAT diagnostic

In `ArduPlane/GCS_Mavlink.cpp` `send_attitude()` (around line 170):
```cpp
send_named_float("YOURNAME", value);  // YOURNAME max 10 chars
```
Visible immediately in MP MAVLink Inspector under `NAMED_VALUE_FLOAT`.

### Show a value as an ESC RPM gauge in MP HUD

```cpp
#if HAL_WITH_ESC_TELEM
    AP::esc_telem().update_rpm(esc_idx_0based, rpm_value, 0);
#endif
```
ESC1 = index 0, ESC2 = 1, etc. ESC3 currently carries `SPN_RPM`.

### Test a new behavior

1. Write a `probe-<feature>.py` in `sitl-launch/` that connects to
   `tcp:127.0.0.1:5762` (SITL's secondary GCS port), exercises the
   feature, and prints results. Pattern after `probe-passive.py` (no
   side effects) or `probe-heli.py` (uses RC override).
2. `PowerShell wsl -d Ubuntu-22.04 -- python3 /mnt/c/Users/dusti/OneDrive/Documents/spin-wing/sitl-launch/probe-yourtest.py`

### Restart SITL cleanly

```bash
# From PowerShell:
wsl -d Ubuntu-22.04 -- bash /mnt/c/Users/dusti/OneDrive/Documents/spin-wing/sitl-launch/clean.sh
wsl -d Ubuntu-22.04 -- bash /home/dustin/run-sitl-spinwing.sh   # run_in_background=true
```

Then wait for SITL via Monitor:
```bash
until wsl -d Ubuntu-22.04 -- bash -c 'ss -tln | grep -q :5762'; do sleep 2; done
```

---

## Memory files (per-project, in user's Claude data dir)

`C:\Users\dusti\.claude\projects\C--Users-dusti-OneDrive-Documents-spin-wing\memory\`:

- `MEMORY.md` — index file, read first
- `user_profile.md` — drone hobbyist, Windows+WSL2, RealFlight+MP, prefers terse direct help, hates being asked clarifying questions when defaults are obvious
- `project_spinwing.md` — high-level project description
- `project_sitl_env.md` — Windows+WSL2 build environment specifics
- `feedback_lean_wsl_setup.md` — for SITL-only, skip `install-prereqs.sh`
- `session_<DATE>.md` — per-session notes (most recent = latest progress)

**Update `session_<TODAY>.md` at the end of each session** with what
landed and what's open, so the next agent can pick up cleanly.

---

## "Restart the project" — full ground-up bring-up

If everything has been lost (new machine, fresh OS install, etc.) follow
the steps in [HANDOFF.md](HANDOFF.md) section "Get going on a new machine".
The whole chain is reproducible — the key files in the repo carry the
parameters and the source carries the code. Only the `.RFX` RealFlight
model is binary and user-owned (must be rebuilt in RealFlight Editor per
the channel-map table in HANDOFF.md).

Setup time on a fresh Windows+WSL2 machine: roughly 30-45 min including
WSL install, ArduPilot clone with submodules, pip3 deps, first build,
and Mission Planner connection test.

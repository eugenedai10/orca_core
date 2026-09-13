# `orca_core` Onboarding Guide

A deep-dive companion to [`CLAUDE.md`](CLAUDE.md). `CLAUDE.md` is the terse "quick facts" file;
this document is the from-scratch, top-to-bottom read for someone new to the package — every
class, every file's role, what every script and test does. Cite-by-name, not by line number:
symbols move, line numbers don't follow them.

## Table of contents

1. [What `orca_core` is](#1-what-orca_core-is)
2. [Big-picture architecture](#2-big-picture-architecture)
3. [Class hierarchy](#3-class-hierarchy)
4. [Core hand-model layer](#4-core-hand-model-layer)
5. [Kinematics](#5-kinematics)
6. [The HTTP API](#6-the-http-api)
7. [Hardware layer](#7-hardware-layer)
8. [Control layer](#8-control-layer)
9. [Maintenance layer](#9-maintenance-layer)
10. [Utils](#10-utils)
11. [Configuration (`config.yaml`)](#11-configuration-configyaml)
12. [Public API surface](#12-public-api-surface)
13. [Scripts](#13-scripts-scripts)
14. [Examples](#14-examples-examples)
15. [Tools](#15-tools-tools-maintainer-only)
16. [Tests](#16-tests-tests)
17. [Cross-cutting conventions](#17-cross-cutting-conventions)
18. [Getting started](#18-getting-started)

---

## 1. What `orca_core` is

`orca_core` is the control package for the ORCA Hand, an open-source dexterous robotic hand. It's
published on PyPI and provides hardware abstraction (motors, encoders, tactile sensors),
calibration/tensioning/assembly routines, and a joint-space control API. Only `orca_core/` itself
ships in the wheel — `scripts/`, `examples/`, `tools/`, `tests/`, `docs/` are repo-only.

It's one of several sibling repos checked out side by side in the wider ORCA superproject
(`orca_ui`, `orca_teleop`, `orca_sim`, `orcahand_description`, `orcahand_hardware`,
`manus-client`), but `orca_core` is the one that matters to open right now: it has no dependency on
any of them, and they all depend on it (as a PyPI package, or — for `orca_ui` specifically — via a
sibling-path `uv` source so local edits here are picked up live). If you're only ever going to read
one of the seven repos, this is the right one; the rest layer UI, teleop, simulation, and CAD on top
of what's described below.

Package layout, at a glance (see later sections for what's actually inside each):

```
orca_core/
├── api/                 # FastAPI HTTP wrapper (early/incomplete) — §6
├── maintenance/         # Interaction-free hardware routines — §9
├── data/                # Packaged content (demo_poses.yaml) — §4
├── control/             # Closed-loop joint control — §8
├── hardware/            # Motor/encoder/tactile hardware interfaces — §7
├── kinematics/          # Rigid transforms, frames, forward kinematics — §5
├── utils/               # Shared utilities — §10
├── models/              # Hand configurations (config.yaml), v1/ and v2/ — §11
├── base_hand.py, hardware_hand.py, hardware_hand_sensing.py   # hand classes — §3, §4
├── hand_factory.py, hand_config.py                            # construction/config — §4
├── calibration.py, joint_position.py, demo_poses.py            # data types — §4
├── version.py, constants.py                                    # shared constants — §4
└── __init__.py          # the public API surface — §12

scripts/    # Thin CLI front-ends: argparse + print + input(); no logic — §13
examples/   # Demo and record/replay scripts built on the public API — §14
tools/      # Maintainer-only tools — §15
tests/      # Unit tests — §16
docs/       # MkDocs site sources — partially stale, verify against code (§17)
```

---

## 2. Big-picture architecture

Everything funnels through one construction path and one motion path.

**Construction:**
```
config.yaml  →  hand_factory.load_hand()  →  a BaseHand subclass instance
                       │
                       ├─ reads config.yaml once, picks a config dataclass
                       │  (OrcaHandConfig or OrcaHandTouchConfig)
                       └─ picks a concrete hand class from a 3-axis matrix:
                            motor family (Dynamixel | Feetech)   → doesn't affect *which class*,
                                                                    only which MotorClient it builds
                            tactile sensing (yes | no)             ┐
                            joint feedback / closed loop (yes|no)  ┘→ picks OrcaHand / OrcaHandTouch /
                                                                       OrcaHandJointFeedback / OrcaHandFull
                            mock (yes | no)                        → swaps in the Mock* twin
```

**Motion, once constructed:**
```
BaseHand.set_joint_positions(...) / get_joint_position()
        │
        ├─ OrcaHand:              joint angles ⇄ motor positions, written straight to MotorClient
        │                         (DynamixelClient | FeetechClient | Mock*)
        │
        └─ OrcaHandJointFeedback: encoder-backed joints are routed through JointLoopThread instead —
                                   a background 100Hz loop reads JointEncoderClient, runs a PI
                                   correction (JointController), and writes the trimmed target to
                                   the same MotorClient. Non-encoder joints (the wrist) still go
                                   straight through the OrcaHand path.
```

Tactile sensing (`OrcaHandTouch`) is a third, independent leg: a `TactileClient` reading fingertip
force/taxel data over its own (or a shared) `HandSerialLink`, with no interaction with motor control
at all except that `OrcaHandFull` has to decide whether the tactile and encoder streams share one
physical serial port.

So a `config.yaml` doesn't just configure a hand — it *picks a class*. `orcahand-right/config.yaml`
gives you a plain `OrcaHand`; `orcahand-full-right/config.yaml` (same joints, same motors, plus a
`sensors:` block and `use_joint_feedback: true`) gives you an `OrcaHandFull` with the identical
motor-space math, plus a live tactile stream and a live closed loop layered on top. Every
combination is a real, named, independently testable class — never an `if self.has_tactile:` branch
sprinkled through one giant class.

---

## 3. Class hierarchy

Verified by C3 linearization (i.e. this is the actual MRO Python computes, not just the inheritance
you'd guess from the `class X(Y):` lines):

```
                         BaseHand(ABC)                        base_hand.py
                              │
                          OrcaHand                             hardware_hand.py
                    ______/   │   \______
                   /          │          \
          OrcaHandTouch  OrcaHandJointFeedback        hardware_hand_sensing.py
                   \          │          /
                 OrcaHandFull(OrcaHandTouch, OrcaHandJointFeedback)
```

`OrcaHandFull`'s MRO is `OrcaHandFull, OrcaHandTouch, OrcaHandJointFeedback, OrcaHand, BaseHand,
ABC, object` — Python resolves tactile before joint-feedback, both before the plain motor-only base.
A dedicated test (`test_full_hand_adds_nothing_beyond_its_capabilities`) asserts `OrcaHandFull`
introduces *zero* new public methods beyond the union of what `OrcaHandTouch` and
`OrcaHandJointFeedback` already expose — the combination class only owns `connect()`/`disconnect()`
orchestration logic, nothing else.

**Mock variants** use a separate mixin, `MockMotorResolutionMixin` (from `hardware_hand.py`),
always placed *before* the real class in each Mock's bases:

```
MockOrcaHand              = MockMotorResolutionMixin, OrcaHand
MockOrcaHandTouch          = MockMotorResolutionMixin, OrcaHandTouch
MockOrcaHandJointFeedback  = MockMotorResolutionMixin, OrcaHandJointFeedback
MockOrcaHandFull           = MockOrcaHandTouch, MockOrcaHandJointFeedback, OrcaHandFull
```

MRO precedence is what makes this work without reimplementing anything: `MockMotorResolutionMixin.connect()`
rewrites `port="mock"`, then calls `super().connect(...)` — which, because of MRO, dispatches to the
*real* capability class's `connect()` (all the shared-link orchestration logic), which in turn calls
construction seams (`_create_motor_client`, `_create_tactile_link`, `_create_encoder_link`, …) that
the Mock leaf classes override to build in-memory clients instead of opening serial ports. "Mock" is
a thin swap of the I/O seams via inheritance order, not a parallel reimplementation of hand logic —
so a mock hand and a real hand run the *exact same* joint-space/calibration/control code.

**`hand_factory._CLASS_MATRIX`** — which class `load_hand()` returns:

| joint feedback | tactile | mock | class |
|---|---|---|---|
| ✗ | ✗ | ✗ | `OrcaHand` |
| ✗ | ✓ | ✗ | `OrcaHandTouch` |
| ✓ | ✗ | ✗ | `OrcaHandJointFeedback` |
| ✓ | ✓ | ✗ | `OrcaHandFull` |
| ✗ | ✗ | ✓ | `MockOrcaHand` |
| ✗ | ✓ | ✓ | `MockOrcaHandTouch` |
| ✓ | ✗ | ✓ | `MockOrcaHandJointFeedback` |
| ✓ | ✓ | ✓ | `MockOrcaHandFull` |

---

## 4. Core hand-model layer

### `base_hand.py` — `BaseHand(ABC)`

The shared joint-space interface for *every* hand backend. Owns config loading/validation, joint
name registration, ROM clamping, linear-interpolated multi-step motion, and named-position
record/replay — all built once here, free to every subclass. Only two methods are abstract seams
subclasses must fill: `_get_joint_positions()` / `_set_joint_positions(joint_pos)`.

Key public methods: `set_joint_positions(joint_pos, num_steps=1, step_size=1e-2)` (coerce → clamp to
ROM → linear-interpolate → write each waypoint), `get_joint_position()`, `pose_from_fractions(fractions)`
(build a pose from ROM fractions, starting at `neutral_position` — this is what `demo_poses.yaml`
feeds into), `register_position`/`remove_position`/`set_named_position`/`play_named_positions`
(named-pose macros), `set_neutral_position()`, `set_zero_position()`.

### `hardware_hand.py` — `OrcaHand(BaseHand)`, `MockMotorResolutionMixin`, `MockOrcaHand`

The motor-only hand — the biggest file in the package. Extends `BaseHand` with the full physical
lifecycle:

- **Connection:** `connect(interactive=True, engage_feedback=True)` — idempotent; resolves the
  serial port (`"auto"` → auto-detect), then the motor family/baudrate (via
  `hardware/motor_resolution.py`'s trial-probe if not pinned in `config.yaml`), opens the client,
  persists what was resolved back to `config.yaml`. `disconnect()` — best-effort torque-disable,
  then always discards the motor client, even on partial failure.
- **Torque / mode / current:** `enable_torque`/`disable_torque(motor_ids=None)` (return failed IDs,
  never raise on an unresponsive motor), `set_max_current(current)`, `set_control_mode(mode,
  motor_ids=None)` — forces the wrist motor into `multi_turn_position` whenever the requested mode
  is `current_based_position`/`current`, since those modes are incompatible with the wrist's ROM.
- **Raw motor reads:** `get_motor_pos`/`get_motor_current`/`get_motor_temp(as_dict=False)`,
  `wait_for_motion(timeout=5.0)` (no-op unless the motor family actually blocks, i.e. Feetech).
- **Joint I/O (the `BaseHand` seam):** `_get_joint_positions`/`_set_joint_positions` convert between
  joint angles and motor positions using `motor_limits_dict`, `joint_to_motor_ratios_dict`,
  `joint_roms_dict`, `joint_inversion_dict`, and per-motor wrap-offset correction for continuous
  rotary encoders. `write_motor_pos(motor_ids, positions)` is a lock-fenced hot-path bypass used by
  `JointLoopThread`.
- **Initialization/calibration:** `init_joints(force_calibrate=False, move_to_neutral=True)` —
  enables torque, sets configured mode/current, calibrates if needed, computes wrap offsets, moves
  to neutral. `is_calibrated(verbose=False, use_joint_feedback=None)`, `calibrate(blocking=True,
  force_wrist=False, joints=None, joint_encoder_client=None, progress_callback=None, persist=None)`
  — delegates to `maintenance/calibration_routine.run_calibration`.
- **Maintenance routines:** `tension(...)`/`jitter(...)` — delegate to `maintenance/tensioning.py`.
  Both, like `calibrate`, run either inline (`blocking=True`) or on a background daemon thread
  (`blocking=False`, via `_start_task`/`stop_task`), driven by the same
  `progress_callback`/`should_stop` pattern described in §9.

`MockMotorResolutionMixin` is the reusable "make any `OrcaHand`-family class a mock" building
block: it synthesizes plausible calibration data in-memory (never writes disk), rewrites the port to
`"mock"`, and swaps `_create_motor_client` for a `MockDynamixelClient`. `MockOrcaHand` is just
`MockMotorResolutionMixin` + `OrcaHand`.

### `hardware_hand_sensing.py` — `OrcaHandTouch`, `OrcaHandJointFeedback`, `OrcaHandFull` (+ Mocks)

Sensing-equipped variants layered on `OrcaHand`, all sharing a consistent shape: construction seams
for links/clients, an attach/open/teardown trio so links can be shared between capabilities, and
`connect()`/`disconnect()` extending the motor-only lifecycle.

- **`OrcaHandTouch(OrcaHand)`** — adds a tactile fingertip sensor over a second serial link.
  `connect()` opens the motor bus then the sensor link, rolling back the motor bus if the sensor
  fails to connect; `connect_sensors_only()` skips the motor bus entirely (for sensor bring-up on
  unpowered motors). Read/stream API: `get_tactile_forces`/`get_tactile_taxels`/`get_tactile_data`,
  `start_tactile_stream`/`stop_tactile_stream`, `zero_tactile_sensors`/`clear_tactile_zero`,
  `get_tactile_configuration`/`get_tactile_stats`/`get_tactile_link_health`, `get_taxel_geometry`.
  Kinematics integration: `kinematics` property (`HandKinematics.load(...)`, requires a v2 model),
  `set_base_pose`/`get_base_pose`, `get_sensor_transforms(frame=...)`, `get_taxel_data(frame=...)` —
  see §5.
- **`OrcaHandJointFeedback(OrcaHand)`** — adds closed-loop joint control from encoder-backed joints.
  Motors stay in `current_based_position`; a `JointLoopThread` trims motor-vs-joint residual at
  ~100 Hz (§8). The wrist is never in the loop — it's driven synchronously via the inherited
  `OrcaHand` path regardless. `connect()` validates the hand's side has a validated encoder-polarity
  table (currently only `"right"`), opens the motor bus, resolves/opens the encoder link, and
  attaches encoders — raising `JointFeedbackConnectError` if there are no calibrated encoder-backed
  joints. `disable_torque`/`enable_torque`/`set_control_mode` are overridden to pause the loop's
  writes for the duration of the round-trip so they don't collide on the shared bus.
  `calibrate`/`tension`/`jitter` refuse to run while the loop is active (they'd fight for the same
  motors) — except `init_joints`, which just skips calibration silently if the loop is already
  running. Loop facade: `set_pid_gains`, `rebase_loop()` (re-anchor after e.g. manually posing the
  hand), `get_measured_joints`, `get_loop_correction`, `get_encoder_link_health`, `get_loop_stats`.
- **`OrcaHandFull(OrcaHandTouch, OrcaHandJointFeedback)`** — combines both without duplicating
  either; owns only the `connect`/`disconnect` orchestration deciding whether tactile and encoder
  streams share one `HandSerialLink` (when port discovery reports they're on the same port) or use
  separate ports/bauds.

All three have `Mock*` counterparts built the same way as `MockOrcaHand` — swap the construction
seams for in-memory equivalents (`MockHandSerialLink`, an internal `_MockEncoderFramePump` daemon
thread feeding synthetic encoder frames so the loop stays "fresh").

### `hand_factory.py` — `load_hand()`, `detect_hand()`

The recommended entry point. `load_hand(config_path=None, calibration_path=None,
model_version=None, model_name=None, mock=False, engage_feedback=True) -> OrcaHand` reads
`config.yaml` once (checks for a `sensors:` key for tactile, `joint_encoder_joints`/
`use_joint_feedback` for feedback), builds the right config dataclass, and indexes the class matrix
from §3. With zero selection arguments and `mock=False`, it calls `detect_hand()` first — which
probes connected hardware over the identity protocol (`ORCA_ID?`/`ORCA_INFO?`) to determine side,
tactile presence, and encoder presence, and picks the matching bundled model.

### `hand_config.py` — config dataclasses

See §11 for the full field table. Three frozen dataclasses: `BaseHandConfig` (joint IDs, ROMs,
neutral position), `OrcaHandConfig(BaseHandConfig)` (adds everything motor/calibration/feedback
related), `OrcaHandTouchConfig(OrcaHandConfig)` (adds the tactile `sensors:` block). All validate
eagerly in `__post_init__` — a malformed `config.yaml` fails at construction, not at first use.

### `calibration.py` — `CalibrationResult`, `JointEncoderCal`

Immutable calibration result types plus their YAML read-side. `JointEncoderCal(enc_at_anchor_count)`
— one raw encoder count captured at a known pose; `joint_angle = polarity * Δenc_wrapped +
anchor_angle_deg`. `CalibrationResult` bundles `motor_limits_dict`, `joint_to_motor_ratios_dict`,
`calibrated`/`wrist_calibrated` flags, and `joint_encoder_calibration_dict`; `from_calibration_path`
loads it (or an all-empty stand-in) from `calibration.yaml`.

### `joint_position.py` — `OrcaJointPositions`

The single typed container every joint-angle API boundary uses — a frozen dict wrapper with a
process-wide registered default joint ordering (set once via `register_joint_names`, called by
`BaseHand.__init__`). `from_dict`/`from_ndarray` (build it), `as_dict`/`as_array`/`as_list`
(read it out, with `NaN`/`None` for missing joints depending on the target type).

### `demo_poses.py` — `load_demo_poses()`

Loader for `data/demo_poses.yaml` — pose data (as ROM *fractions*, not raw angles, so one preset
works across hand models with different ranges), shared by the bundled examples and external
front-ends. Deliberately **not** re-exported from the top-level package — "demo content is not part
of the hand-control API." Import it explicitly: `from orca_core.demo_poses import load_demo_poses`.

### `version.py` / `constants.py`

`version.py`: `LATEST_VERSION = "v2"` — the default packaged model version, re-exported at top
level. `constants.py`: a grab-bag of shared naming/protocol constants, no classes — joint/finger
naming (`FINGER_NAMES`, yaml key constants), motor families (`DYNAMIXEL`, `FEETECH`,
`SUPPORTED_MOTOR_TYPES`), USB vendor IDs for port auto-detection, the connector-board identity
protocol constants, control-mode names + `MODE_MAP` + `WRIST_MODE_VALUE`, timing defaults
(`NUM_STEPS`, `STEP_SIZE`), and `MOTOR_BAUD_RATES` (the candidate list `motor_resolution.py` probes).
Not re-exported at the package's top level at all.

---

## 5. Kinematics

Pure-numpy, no hardware dependency — backs `OrcaHandTouch`'s tactile-frame API. Only
`OrcaHandTouch`/`OrcaHandFull` reach into this module.

- **`kinematics/transforms.py`** — `Transform`, an immutable SE(3) rigid transform (4×4 matrix
  under the hood): `identity()`, `from_xyz_rpy`, `from_rotation_translation`, `@` for composition,
  `inverse()`, `apply_to_points`/`apply_to_vectors`. `rotation_about_axis(axis, angle)` — Rodrigues'
  formula.
- **`kinematics/frames.py`** — the five named frames, innermost first: `SENSOR` (native taxel
  frame) → `FINGERTIP` (distal link) → `PALM` (carpals, moves with wrist) → `BASE` (static hand
  root) → `WORLD` (base + a user-supplied pose).
- **`kinematics/hand_kinematics.py`** — `HandKinematics.load(hand_type)` loads a packaged,
  URDF-derived kinematic chain (`kinematics/data/v2_kinematics.yaml`); `fingertip_poses(joint_pos_deg,
  in_frame=BASE)` and `sensor_poses(...)` compute forward kinematics for all five fingers.

---

## 6. The HTTP API

`api/api.py` — an early/incomplete FastAPI wrapper around a single global `OrcaHand` instance. No
auth, one hand, minimal validation beyond what `OrcaHand` itself does. It only ever constructs a
plain `OrcaHand` (never routes through `load_hand()`), so there are no tactile/joint-feedback
endpoints today.

| Endpoint | Behavior |
|---|---|
| `POST /config` | Rebuild the global hand from a new `config_path` |
| `POST /connect` / `POST /disconnect` | `connect(interactive=False)` / stop any task + disconnect |
| `GET /status` | `{"connected", "calibrated"}` |
| `POST /torque/enable` / `POST /torque/disable` | per-motor torque toggle |
| `POST /current/max` | `set_max_current` |
| `GET /motors/position` / `/current` / `/temperature` | raw motor reads |
| `GET /joints/position` / `POST /joints/position` | `get_joint_position` / `set_joint_positions` |
| `GET /calibrate/status` / `POST /calibrate` | poll / kick off a background (non-blocking) calibration |

Two locks matter: `_hand_init_lock` guards lazily building the global hand; `_hand_lock` serializes
`/connect`/`/disconnect`/`/config`/`/calibrate` against each other (non-blocking acquire — an
overlapping call gets HTTP 409, not a queue). Read endpoints stay lock-free.

---

## 7. Hardware layer

### `hardware/motor_client.py` — `MotorClient(ABC)`

The contract every motor-family driver implements, so calling code never branches on motor family.
Class attrs describe the family (`motor_type`, `factory_default_id`, `factory_default_baudrate`,
`baud_rate_map`, `requires_unpowered_hotplug`). Abstract methods: `is_connected`, `connect`,
`disconnect`, `set_torque_enabled(motor_ids, enabled, retries=3, retry_interval=0.25) -> list[int]`
(returns failed IDs, never raises), `set_operating_mode(motor_ids, mode)`,
`read_position_velocity_current() -> MotorRead`, `read_temperature()`, `write_desired_pos`,
`write_desired_current`. Concrete/overridable: `last_read_ok` (default `True`),
`wait_for_motion_complete` (no-op default), `requires_offset_calibration` (default `False`),
`calibrate_offset` (no-op default). Optional provisioning methods (`scan_for_motors`,
`change_motor_id`, `change_motor_baudrate`) raise `NotImplementedError` unless a family overrides
them — driven by `maintenance/motor_chain.py`.

### `hardware/motor_factory.py`

`motor_client_class(motor_type) -> type[MotorClient]` and `create_motor_client(motor_type,
motor_ids, port, baudrate)` — the single dispatch point from a `motor_type` string to its client
class, with lazy per-family imports (a Dynamixel-only hand never imports the Feetech SDK).

### `hardware/motor_resolution.py`

Connect-time auto-detection when `config.yaml` doesn't pin `motor_type`/`baudrate`.
`trial_probe(config, port)` tries `{dynamixel, feetech} × candidate baudrates` until one client's
`probe()` succeeds. `persist_resolved_driver(existing, resolved)` writes only the fields that were
missing back to `config.yaml`, atomically, logging (not raising) on a read-only install.

### `hardware/dynamixel_client.py` — `DynamixelClient(MotorClient)`

Production driver for Dynamixel X-series motors (Protocol 2.0, via `dynamixel_sdk`). All I/O is
serialized by an `RLock` (re-entrant because hardware-alert recovery re-enters from inside a locked
path); failed transactions flush the OS receive buffer before releasing the lock. Reactive
overload-alert handling: `handle_packet_result` inspects the error byte's Alert bit inline (no
polling needed) and `_handle_hardware_alert` reboots the affected motor and restores its mode/torque.
Bulk reads (`DynamixelPosVelCurReader`, `DynamixelTempReader`) fall back to a rate-limited per-motor
retry on partial/total failure rather than propagating a stale read. `connect()` never touches
torque — connecting must never make the hand move.

### `hardware/feetech_client.py` — `FeetechClient(MotorClient)`

Production driver for Feetech SCServo motors (SMS/STS family, e.g. HLS3930/HLS3915 — both driven
through the same `sms_sts` protocol class, no per-model branching anywhere in this file). Same
bus-lock contract as Dynamixel. Two behavioral differences worth knowing: `waits_for_motion = True`
(unlike Dynamixel, `wait_for_motion_complete` actually polls and blocks), and
`requires_offset_calibration = True` with a real `calibrate_offset` implementation (an EEPROM
`INST_OFSCAL` write) — Feetech motors need this, Dynamixel doesn't.

Writing an EEPROM-resident register (`SMS_STS_MODE`, `SMS_STS_ID`, `SMS_STS_BAUD_RATE`) needs
`unLockEprom(motor_id) → write → LockEprom(motor_id)` or Feetech's protection register rejects the
write with `error=2` — the vendored SDK's own memory-map comments (`hardware/feetech/sms_sts.py`)
place `SMS_STS_MODE` (33) in the EEPROM (read-write) block, `SMS_STS_TORQUE_ENABLE` (40)/
`SMS_STS_LOCK` (55) in the volatile SRAM block (so torque-enable writes are never gated by this
lock, only the EEPROM-resident registers are). **Currently this wrap is only present at the
`change_motor_id`/`change_motor_baudrate` call sites** — the `SMS_STS_MODE` write in `connect()`'s
startup "force servo mode" loop and in `set_operating_mode()` is unwrapped, so expect `error=2` on
real Feetech hardware for any connect-time mode enforcement or control-mode change until that's
fixed. Whichever way that's resolved, the fix is never model-specific: `feetech_client.py` never
branches on motor model (wrist `HLS3930` vs. finger `HLS3915`), it always drives the same generic
`sms_sts` register map for every motor on the bus.

#### Feetech operating modes

The `SMS_STS_MODE` register (address 33) actually has **4** documented hardware values — this
codebase's own code comment ("Feetech only supports Mode 0/Mode 1") describes what this *driver*
uses, not the full chip capability:

| Value | Feetech calls it | Does `orca_core` ever write this? |
|---|---|---|
| `0` | Position (servo mode) | Yes — the only mode this driver drives to |
| `1` | Constant speed (wheel mode) | Yes — the only other mode this driver drives to |
| `2` | PWM (open-loop duty cycle) | Never used |
| `3` | Step servo | Never used |

`orca_core`'s generic `control_mode` API (`OrcaHand.set_control_mode`, 5 named values via
`constants.MODE_MAP`, written mode-first for Dynamixel) squashes every value down to just `0` or
`1`:

| Generic mode (`MODE_MAP` value) | On Feetech |
|---|---|
| `current` (0) | Not driven → warns, falls back to servo mode (0) with whatever torque value is already cached |
| `velocity` (1) | Native match → wheel mode (1) — the only value that maps 1:1, no warning |
| `position` (3) | Native match → servo mode (0), no warning (it's the natural mapping) |
| `multi_turn_position` (4) | Not driven → warns ("using servo mode (limited to 360°)"), falls back to servo mode (0); no continuous-rotation tracking is possible at all (mode `3`/Step servo might offer something closer to this on real hardware, but this driver never uses it) |
| `current_based_position` (5) | Falls back to servo mode (0) — **deterministically never wheel mode**, since `feetech_mode = 1 if mode == 1 else 0` and `current_based_position` is numeric value `5`, not `1`. The current ceiling is *intended* to be pushed separately through `write_desired_current()` — see the safety callout below, this part is unconfirmed |

`OrcaHand.set_control_mode` always force-overrides the wrist motor specifically into
`multi_turn_position` (4) whenever the rest of the hand goes to `current_based_position`(5) or
`current`(0) — "incompatible with the wrist joint's range of motion." On a Feetech wrist that
override still lands on servo mode (0) capped at 360° (mode `4` isn't driven here either), with a
warning logged every time. Harmless in practice (the v2 wrist ROM, `[-65°, 35°]`, fits comfortably
inside 360°) but it means a Feetech wrist never gets the continuous rotation a Dynamixel wrist can.

Also Feetech-specific: `POSITION_DIRECTION = -1` corrects Feetech's inverted raw rotation sense so
`OrcaHand` sees one motor-agnostic sign convention regardless of family.

#### ⚠️ Current limiting on Feetech is unconfirmed — read before trusting it during tension/calibration

`write_desired_current()` maps mA to a 0–1000 unit and caches it as `self._default_torque`, which
becomes the `torque` parameter of the next `write_positions_sync()` call. That parameter is written
into a 2-byte field at addresses 44–45 of the sync-write packet. The vendored SDK's own register
table (and Feetech's official upstream SDK, `ftservo/FTServo_Python` — this vendored copy matches it
byte-for-byte, including this exact naming inconsistency) labels that address range
`SMS_STS_GOAL_TIME_L/H` — **"time to reach goal position, in milliseconds,"** not a torque or current
limit. Whether that field is actually dual-purposed as a torque limit on real HLS3915/HLS3930
firmware, or whether `orca_core`'s "torque" value is silently being interpreted as a movement
duration instead, is **not resolved** — it isn't confirmed anywhere in this codebase, and Feetech's
own SDK is internally inconsistent about it (same naming mismatch exists in their official code).

Separately, Feetech servos have their *own* independent onboard overload/stall protection, driven by
a different set of registers this codebase never touches at all (reported values from community
sources, not verified against this codebase or an HLS-specific datasheet: a "Protection Current,"
a "Protective Torque," an "Overload Torque," and an "Over-current Protection Time," each with a
fixed factory-default threshold). If that's accurate, then **`calibration_current`/
`wrist_calibration_current`/`max_current` may not be capping the force a Feetech motor applies when
stalled at all** — the servo's own factory-default overload thresholds would be doing that instead,
regardless of what value `orca_core` sends.

This matters most during **tension** (§9) and **calibration** (§9): both stages depend on a Feetech
motor being able to stall gently at low force — tension so a human can safely adjust tendons by
hand while the motor holds, calibration so driving into a hardstop can't damage the mechanism. If
the "current cap" isn't actually reaching the servo, a stalled Feetech motor could be pushing with
whatever force the chip's own default overload protection allows, not the lower value `orca_core`
intends. **Verify this empirically on your actual hardware before relying on it** — e.g. command a
low `calibration_current`, physically block a motor from reaching its target, and measure actual
current draw/holding force, rather than trusting either this codebase's comments or third-party
Feetech documentation on the point. This is unresolved, not a "known bug with a known fix."

### `hardware/mock_dynamixel_client.py` — `MockDynamixelClient(MotorClient)`

In-process simulation of `DynamixelClient` for tests/`MockOrcaHand`. Mirrors the real client's
method surface and lifecycle contracts (same `OPEN_CLIENTS` registry, same "torque untouched on
connect" rule) but keeps state in plain dicts — while still constructing real `dynamixel_sdk`
`PortHandler`/`PacketHandler` objects internally, so the same SDK codec logic runs in tests. Not
reachable through `motor_factory.py` (only real families are); wired in directly by
`MockMotorResolutionMixin`/test fixtures.

### `hardware/feetech/` — vendored third-party SDK

A vendored copy of Feetech's "SCServo" Python SDK, flat-re-exported via `feetech/__init__.py`:
protocol-level constants (`scservo_def.py`), serial port ownership (`port_handler.py`), packet
framing/checksums (`protocol_packet_handler.py`), batched multi-motor transactions
(`group_sync_read.py`/`group_sync_write.py`), and three servo-family register maps (`sms_sts.py` —
the one `feetech_client.py` actually uses; `hls.py` and `scscl.py` — vendored but unused). This is
low-level plumbing only — `feetech_client.py` builds the real `MotorClient` contract (bus-lock
thread-safety, retry policy, position-direction correction, offset-calibration semantics) on top of
it; the vendored SDK itself has no opinion on any of that.

### `hardware/hand_serial_link.py` / `mock_hand_serial_link.py`

`HandSerialLink` owns the physical serial port to the hand's connector board and demultiplexes its
AA-framed wire protocol into two channels: synchronous request/response (`AA 55`, for register
reads/writes) and asynchronous auto-stream broadcasts the firmware emits on its own clock (`AA A9`
encoder frames, `AA 56` tactile frames). One background thread (`_demux_loop`) does all reading;
callers either block on `send_register_request` or register a handler for a given frame type via
`register_frame_handler`. `LinkStats` tracks diagnostic counters (frames routed/dropped/bad-checksum,
handler errors, resync events) consumed by `hardware/sensing/health.py`.

`MockHandSerialLink` is a genuine subclass overriding *only* the four I/O-seam methods
(`_open_serial`/`_close_serial`/`_serial_write`/`_serial_read`) — the demuxer thread, handler
dispatch, and request/response locking are the same production code, so there's no parallel
protocol implementation that can drift from the real one. Test-facing API: `feed_bytes`,
`simulate_port_death`, `set_response_provider` (round-trip `send_register_request` synchronously in
a test).

### `hardware/joint_encoder_client.py`

`JointEncoderClient` consumes the always-on `AA A9` joint-encoder auto-stream. `start_stream()`
blocks until the first frame arrives (or raises `EncodersNotAvailableError` on timeout);
`get_latest()` returns the cached `EncoderReading`. Also hosts the anchor-sampling math used during
calibration: `average_anchor_count` (circular mean of 14-bit encoder counts, correct across the
wrap) and `sample_anchor_count_from_client` (polls until enough *distinct, chip-valid* samples
arrive, rejecting parity/angle-error-flagged frames without counting them).

### `hardware/tactile_client.py`

`TactileClient` consumes both the tactile register protocol (config reads/writes) and the `AA 56`
auto-stream, over a shared `HandSerialLink`. Handles sensor discovery/config caching, resultant-force
and per-taxel decoding, zeroing-offset capture (`capture_taxel_offsets`), and a self-healing
"re-arm" mechanism: if the stream's gone stale longer than a threshold, a background thread
re-writes the stream-enable register (handles a firmware reset that silently clears it), guarded by
a generation counter so a stopped/restarted stream doesn't get stomped by a stale re-arm.

### `hardware/sensing/` — supporting library

| File | Role |
|---|---|
| `framing.py` | The shared LRC checksum used by every AA-framed protocol |
| `constants.py` | Every shared sensing constant: baud defaults, register addresses, wire headers, encoder bit layout, per-side polarity tables, all link/stream timing constants |
| `encoder_protocol.py` | Pure codec: `parse_encoder_frame`, parity check, `encoder_to_joint_angle` (wraparound-correct) |
| `types.py` | Frozen dataclass containers: `ResultantReading`, `TaxelReading`, `TactileReading`, `EncoderReading`, `TaxelData`, `LinkHealth` |
| `taxel_geometry.py` | Loads static per-taxel sensor positions from packaged per-model YAML |
| `tactile_protocol.py` | Pure codec for the tactile register + auto-stream wire formats (encode/decode, no I/O) |
| `health.py` | Turns raw stream counters into pass/fail verdicts: `EncoderStreamHealth`, `detect_wiring_mismatch`, `diagnose_encoder_link` |
| `serial_discovery.py` | Port auto-discovery: `discover_sensing_ports`/`resolve_sensing_ports`, the `ORCA_ID?`/`ORCA_INFO?` identity protocol, encoder-stream passive detection |
| `tactile_mock.py` | Stateful `AA 55`/`AA 56` responder driving a `MockHandSerialLink` with synthetic tactile data |

---

## 8. Control layer

### `control/joint_loop.py` — `JointLoopThread`

The host-side closed-loop controller: reads absolute joint-encoder angles, computes a PI trim on top
of a commanded target, maps the corrected angle to a motor position, writes it — at a fixed rate
(default 100 Hz), with a tiered encoder-freshness watchdog and a loop-jitter e-stop.

Per-cycle state machine (`step_once`), checked in order against time-since-last-encoder-frame:

| Tier | Threshold | Behavior |
|---|---|---|
| `WATCHDOG_WARN_MS` | 15 ms | Rate-limited warning log only |
| `WATCHDOG_HOLD_MS` | 50 ms | Freeze the PI integrator (P term still runs) |
| `WATCHDOG_HOLD_BASE_MS` | 200 ms | Drop the PI trim entirely — write only the base target |
| `WATCHDOG_STOP_LOOP_MS` | 1000 ms | E-stop — the loop stops; the motor's own internal PID holds the last commanded position |

`start()` primes calibration snapshot + anchors to the current pose (bumpless start — no lurch).
`rebase()` re-anchors a *running* loop without a restart (e.g. after manually posing the hand with
torque off). `pause_writes()`/`resume_writes()` fence the loop off the bus so another motor
operation (torque toggle, mode change) can't interleave with its writes. A separate jitter monitor
tracks consecutive slow/pathological cycle streaks and e-stops after 5 consecutive pathological
cycles (loop period far exceeding target).

### `control/joint_controller.py` — `JointController`

A vectorized, thread-safe PI controller (no D term — the inner motor PID is already damped, and D
on a quantized 100 Hz encoder would mostly amplify noise) with conditional-integration anti-windup:
the integrator only accumulates where the output isn't both saturated *and* being pushed further
into saturation by the current error. `step(target_deg, measured_deg, dt)` returns the correction;
`set_gains(Kp, Ki, correction_max_deg, i_clamp_deg)` validates and broadcasts scalars/arrays
atomically (never leaves the controller half-configured on a bad input).

### `control/constants.py`

The tuning-knob table for the whole loop: `DEFAULT_LOOP_HZ=100`, the four watchdog tiers above,
`DEFAULT_KP=1.0`/`DEFAULT_KI=12.0`/`DEFAULT_CORRECTION_MAX_DEG=60.0`/`DEFAULT_I_CLAMP_DEG=15.0`, and
the jitter-monitor ratios/streak-length constants.

---

## 9. Maintenance layer

Interaction-free hardware routines — "interaction-free" meaning they never call `print`/`input`
themselves; they report and prompt through callbacks so a terminal script, a GUI, or (per the
routines' own docstrings) a future web front-end can all drive the *identical* code.

### The `progress_callback` / `prompt_callback` / `should_stop` pattern

```python
ProgressCallback = Callable[[dict], None]   # fire-and-forget progress event; never blocks
PromptCallback   = Callable[[dict], None]   # BLOCKS until the operator has physically acted
ShouldStop       = Callable[[], bool]       # polled for cooperative cancellation
```

- `progress_callback({"event": ..., **payload})` — every routine reports what it's doing by calling
  this with an `"event"`-tagged dict. All three modules' internal `_emit` helpers catch and log any
  exception the callback raises — "a misbehaving front-end callback must not abort the operation."
- `prompt_callback({"action": ..., **payload})` — used only where a human must physically do
  something before the routine can continue (assembly-time motor hot-plugging). Unlike
  `progress_callback`, this call is expected to *block* until the action is done. If a routine needs
  one and none was supplied, it raises rather than hanging forever.
- `should_stop()` — polled between hardware commands for cooperative cancellation.

A terminal script plays both roles directly (`scripts/configure_motor_chain.py`'s `on_progress`
switches on `event["event"]` to print human-readable lines; `on_prompt` does
`input("Press Enter when the motor is connected...")` — the literal blocking call). A GUI would
update widgets instead of printing, and show a modal instead of blocking on `input()` — the routine
itself is unaware of and unaffected by which kind of front-end is driving it.

`OrcaHand.calibrate()`/`.tension()`/`.jitter()` bridge this to background execution: they always
wire `should_stop=self._task_stop_event.is_set`, and offer `blocking: bool` to run the routine
inline or via `_start_task`/`stop_task` on a daemon thread — so `hand.calibrate(blocking=False,
progress_callback=my_gui_update)` gives a GUI a responsive, cancellable operation using the exact
same `run_calibration()` a terminal script calls with `blocking=True`.

### Control mode across the hand lifecycle

Both `maintenance/tensioning.py` and `maintenance/calibration_routine.py` (and `OrcaHand.init_joints`)
switch `control_mode` for their own purposes and restore it afterward. `orca_core` sends the same
generic `control_mode` string regardless of motor family — but the two families resolve it to
different hardware modes. First, the direct mapping (this is the same information as §7's Feetech
table and the Dynamixel mode-value table, laid out side by side for comparison):

| `control_mode` (`MODE_MAP` value) | Dynamixel operating mode | Feetech `SMS_STS_MODE` |
|---|---|---|
| `current` (0) | `0` — Current Control (real current control) | `0` — Position/servo. Current control isn't driven on Feetech; falls back, warns |
| `velocity` (1) | `1` — Velocity Control | `1` — Constant speed/wheel. The one native 1:1 match |
| `position` (3) | `3` — Position Control (no current ceiling) | `0` — Position/servo. Native match, no warning |
| `multi_turn_position` (4) | `4` — Extended Position Control (true multi-turn, no 360° limit) | `0` — Position/servo, capped at 360°. Multi-turn isn't driven on Feetech; falls back, warns |
| `current_based_position` (5) | `5` — Current-based Position Control (position + real hardware current ceiling) | `0` — Position/servo. **Never** `1`/wheel — deterministic in code (`feetech_mode = 1 if mode == 1 else 0`, and `5 != 1`). The current ceiling is sent separately via `write_desired_current`; whether it's actually enforced is the unconfirmed part, not which mode it's in |

Every `control_mode` Feetech is asked for collapses to just `SMS_STS_MODE` `0` or `1` — Dynamixel is
the only family where `current`/`position`/`multi_turn_position`/`current_based_position` are truly
four distinct hardware modes. Now the same mapping, applied stage by stage:

| Stage | `control_mode` sent | Current/torque value sent | On Dynamixel | On Feetech |
|---|---|---|---|---|
| Motor-chain assembly (`configure_motor_chain.py`) | *(never touched)* | — | Untouched | Untouched — only `ID`/`BAUD_RATE` registers are written |
| `connect()` | *(defensive, not a "mode" per se)* | untouched | No-op (torque/mode are left as-is on connect) | Every motor unconditionally forced to `SMS_STS_MODE=0`, "in case motors were left in wheel mode from a previous session" |
| Tension — winding | `current_based_position` → mode `5` | `calibration_current` | Mode `5`: **hardware-enforced current ceiling** — the actuator genuinely stops increasing torque once the limit is hit, even short of the target position | Mode `0`. Current value is sent via `write_desired_current`, but **whether it actually caps stall force is unconfirmed** — see the ⚠️ callout in §7 |
| Tension — ramp | `current_based_position` → mode `5` | `max_current` ramped to `0` over 20 steps | Same hardware ceiling, ramped down — reliably reduces holding force to zero | Mode `0`, same ramp sent; per the ⚠️ callout, whether output force actually tracks it is unconfirmed |
| Tension — holding | `current_based_position` → mode `5` | `max_current` (full) | Motor holds rigidly, current capped at `max_current` by hardware | Mode `0`; current cap during the hold is subject to the same ⚠️ caveat |
| Calibration (drive-to-hardstop) | `current_based_position` → mode `5` | `calibration_current`, or `wrist_calibration_current` for the wrist | Hardware guarantees the drive-into-hardstop can't exceed `calibration_current` | Mode `0`. **Read the ⚠️ callout in §7 before trusting this on real hardware** — the low-current safety margin calibration depends on is not confirmed to be enforced |
| Neutral move (`init_joints`/`set_neutral_position`) | temporarily `position` → mode `3` | uncapped | Mode `3`: true uncapped position, no current ceiling at all | Mode `0` — **the identical register value** mode `5` already resolves to; the only real change is that no new torque value is pushed this cycle, so whatever was last cached carries over |
| Normal operation | `config.yaml`'s `control_mode` (default `current_based_position` → mode `5`) | `max_current` | Reliable hardware current ceiling during everyday compliant grasping | Mode `0`; same ⚠️ caveat applies during normal grip, not just bring-up — a "gentle grip" may not be as gentle as intended |

**Why Dynamixel is solid here:** its mode `5` is a real, well-documented hardware feature —
position control with an explicit, continuously-enforced `Goal_Current` ceiling; if something
blocks the joint, the actuator stops pushing harder once that ceiling is hit, full stop, regardless
of the position error. This is exactly the "stall gently" behavior tension and calibration are
designed around, and on Dynamixel it's guaranteed by the chip itself — a real mode change, not an
emulation.

**Why Feetech needs verification, not trust:** there is no equivalent *mode* to switch to — Feetech
never leaves mode `0` for any of this. The current limiting is attempted through a side channel
(`write_desired_current`) into a register field Feetech's own official SDK independently labels
"Goal Time" (movement duration), not a documented torque/current limit. Separately, Feetech servos
have their own onboard overload protection with fixed factory-default thresholds that `orca_core`
never touches or overrides — so even if the "torque limit" write is a no-op, *some* protection
likely exists, just not the one `calibration_current`/`max_current` is meant to control. **Before
running `tension.py` or `calibrate.py` on a real Feetech hand, confirm empirically** (block a motor
by hand at a low `calibration_current` and feel/measure the actual holding force) rather than
assume the low-current intent in this table is what the servo is actually doing.

### `maintenance/motor_chain.py`

Drives hand-assembly: assigning each motor its final ID and baud rate, one motor at a time as it's
physically plugged in (motors ship at a shared factory default — Feetech's is `id=1, baud=1,000,000`
— so two fresh motors can never coexist on the bus until reprogrammed). `MotorChainPlan` builds the
target layout from `config.yaml`'s `joint_to_motor_map`: `finger_ids` (every motor except the wrist,
sorted **descending**) followed by the wrist **last** in configuration order — the wrist isn't
necessarily the highest ID, it's just always configured last in the daisy-chain sequence.

`configure_motor_chain(plan, progress_callback, prompt_callback, should_stop)` is the top-level
driver:
1. **Prescan** — finds motors already at their target ID, so a re-run resumes instead of restarting;
   anything that doesn't fit the expected prefix aborts the run rather than guessing.
2. **Per remaining motor, in order:** wait for it to appear — Dynamixel is hot-pluggable, so this
   just polls the bus; Feetech sets `requires_unpowered_hotplug=True`, so the routine instead waits
   for the port to *disappear* (bus powered off), prompts you to connect the next motor
   (`prompt_callback`, blocking), then waits for the port to *reappear*.
3. **Verify the model** answering at the factory default matches what's expected (a wrong-model
   motor is reported and re-prompted, not aborted).
4. **Configure it** — baud rate changed *before* ID, deliberately: an interruption mid-step leaves
   the motor at a still-recoverable (ID, baud) pairing rather than stranded somewhere unvisited.
5. **Verify the chain** — re-scans to confirm exactly the expected IDs answer now.

Never touches `SMS_STS_MODE`/operating mode at all (see the table above). `reset_all_motors`/
`change_all_baudrates` are bulk variants applying one action to every motor currently on the bus in
a single pass, rather than walking the assembly sequence.

### `maintenance/calibration_routine.py`

`run_calibration(hand, force_wrist=False, joints=None, joint_encoder_client=None, ...)` finds each
joint's true mechanical hardstops and the joint-angle↔motor-position conversion ratio everyday
control depends on. Switches to `current_based_position` at `calibration_current` first. Then walks
`config.yaml`'s `calibration_sequence` (ordered `{joints: {name: "flex"/"extend"}}` steps); per
joint in a step:
- Re-enables torque for just that motor; sets current to `calibration_current`, or
  `wrist_calibration_current` specifically for the wrist (gentler).
- Drives in small increments (`calibration_step_size`) in the flex/extend direction (adjusted for
  the joint's configured inversion) until the read-back position stops changing (within
  `calibration_threshold`) for `calibration_num_stable` consecutive reads — a real hardstop, not a
  stale bus read.
- If joint-encoder feedback is enabled, samples the raw encoder count right here, **while still
  under load** — this becomes the joint's calibration "anchor" for closed-loop control.
- **Releases torque**, reads the now-relaxed position as the recorded limit (a still-tensioned
  reading would be wrong), and — since Feetech `requires_offset_calibration=True` — re-runs the
  motor's own offset-calibration command, then re-engages torque.

Once both flex and extend limits for a joint are captured, computes `joint_to_motor_ratio =
Δmotor / Δjoint`. **Persists to `calibration.yaml` after every single step**, atomically — an
interrupted run (Ctrl+C, crash) never loses progress already made.

### `maintenance/tensioning.py`

`run_tension(hand, move_motors=True)` takes up tendon slack, then holds the hand rigid so an
operator can physically tension/crimp tendons by hand:
1. Switches to `current_based_position`.
2. If `move_motors` (default): **winding** — every non-wrist motor is nudged `±0.1 rad` at
   `calibration_current` repeatedly (one direction, then the other) until the max per-motor position
   change over a rolling 0.1s window drops below `0.01 rad` for a full second (tendon's taut, motor's
   stalled), capped at 20s per direction. Then **ramp** — re-engage at full `max_current`, linearly
   step it to `0` over 20 steps (~1s) so the tendon doesn't snap back, briefly disable torque.
3. **Holding** — re-engage torque at full `max_current` and idle-loop until `should_stop` — the
   actual window where the manual tensioning work happens, with the motors holding position firmly.
4. On *any* exit path — clean stop or a crash — `max_current`, `control_mode`, and torque are always
   restored (cleanup failures are logged, not swallowed; the original error still propagates).

`run_jitter(hand, amplitude=5.0, frequency=10.0, duration=3.0)` — oscillates motors sinusoidally
(amplitude hard-capped at 10°) to help seat tendons, restoring exact start positions afterward.

### `neutral.py`'s two-call move (`OrcaHand.init_joints` + `set_neutral_position`)

`scripts/neutral.py` calls `hand.init_joints(force_calibrate=...)` then `hand.set_neutral_position()`.
`init_joints`: enables torque, sets the configured `control_mode`/`max_current`, calibrates if
needed, computes wrap-offsets, then — since `move_to_neutral` defaults `True` — **temporarily
switches to plain `position` mode**, moves every joint to `neutral_position` over `NUM_STEPS` (50)
interpolated steps, then **switches back** to the configured mode. `set_neutral_position()` right
after does the identical temporary-`position`-mode dance around the same interpolated move — harmless
redundancy here since `init_joints` already got there, but the method is also called standalone
elsewhere (e.g. after `calibrate.py`/`tension.py`, or by a GUI) without going through `init_joints`.

Why the temporary mode switch: the recorded `neutral_position` values assume a firm, uncapped move;
under `current_based_position`'s lower current ceiling, the motor might not have enough force to
fully overcome tendon tension/friction and actually reach that pose. On Dynamixel this matters a lot
(`position` mode truly has no current ceiling there). On Feetech it's a smaller effect per §7's mode
table — both `position` and `current_based_position` write the same servo-mode-0 register value, so
what actually changes is that no new torque limit gets pushed through `write_desired_current` during
that window; the motor drives with whatever torque ceiling was last cached (sticky until explicitly
changed again).

---

## 10. Utils

### `utils/utils.py`

`get_model_path(model_path=None, model_version=None, model_name=None)` — the central resolver every
script/example uses to find a `config.yaml`: bundled default, absolute path, or a relative
name/path tried against several candidate locations, with a cross-version fallback search. YAML
helpers: `write_yaml_atomic` (whole-document atomic replace — temp file + `fsync` + `os.replace`,
used for calibration persistence) vs. `update_yaml` (single-key, non-atomic in-place rewrite); `read_yaml`
returns `{}` (not `None`) on a missing file — worth remembering, since some callers rely on `read_yaml(...)
or {}` patterns elsewhere. Port discovery: `auto_detect_port`, `motor_type_for_port`,
`find_single_usb_serial_port`, and an interactive `curses` port-picker (`get_and_choose_port`).

### `utils/cli.py`

The shared plumbing nearly every script uses: `add_hand_arguments(parser)` (adds the common
`config_path`/`--mock` args), `create_hand(config_path, use_mock)`, `connect_hand(hand)` (connect +
print + raise on failure), `shutdown_hand(hand)` (best-effort stop-task + disconnect).

---

## 11. Configuration (`config.yaml`)

`config.yaml` is the source of truth for a hand. Every field, with its default and meaning:

| Field | Default | Meaning |
|---|---|---|
| `type` | — | `"left"`/`"right"` |
| `joint_ids` | — | ordered list of joint names |
| `joint_roms` | — | `{joint: [min_deg, max_deg]}` |
| `neutral_position` | — | `{joint: angle_deg}` |
| `motor_type` | `None` (auto-detect) | `"dynamixel"` / `"feetech"` |
| `port` | `"auto"` | serial port |
| `baudrate` | `None` (auto-probe) | motor bus baud rate |
| `max_current` | `300` mA | motor current ceiling |
| `control_mode` | `current_based_position` | see `CONTROL_MODES` in `constants.py` |
| `motor_ids` | `[]` | ordered motor IDs |
| `joint_to_motor_map` | `{}` | `{joint: ±motor_id}` — negative = inverted |
| `calibration_current` / `wrist_calibration_current` | `200` / `100` mA | current during calibration sweep |
| `calibration_step_size` / `_step_period` / `_threshold` / `_num_stable` | — | hardstop-detection tuning |
| `calibration_sequence` | `[]` | ordered `{joints: {name: "flex"/"extend"}}` steps |
| `use_joint_feedback` | `None` (defers to `has_joint_encoders`) | tri-state override |
| `joint_encoder_joints` | `None` | list of encoder-backed joints, or `["all"]` |
| `encoder_serial_port` / `encoder_baudrate` | `"auto"` / 2,000,000 | encoder link |
| `sensors.port` / `.baudrate` / `.finger_to_sensor_id` | `"auto"` / `"auto"` / thumb=0..pinky=4 | tactile link (`OrcaHandTouchConfig` only) |

### v1 vs. v2

- v1 uses `baudrate: 3000000`/`max_current: 400`; v2 uses `1000000`/`300`.
- Thumb joint naming differs: v1 is `thumb_mcp, thumb_abd, thumb_pip, thumb_dip`; v2 is
  `thumb_cmc, thumb_abd, thumb_mcp, thumb_dip` (different anatomical naming for the thumb chain
  specifically — the other four fingers use `abd`/`mcp`/`pip` in both).
- v1 ships only bare left/right (motor-only); v2 adds four sensing variants per side.

### v2 capability variants

Each is the base `orcahand-{side}/config.yaml` plus an additive block:

| Model name | Adds | Resulting class |
|---|---|---|
| `orcahand-{side}` | — | `OrcaHand` |
| `orcahand-touch-{side}` | `sensors:` block | `OrcaHandTouch` |
| `orcahand-joint-{side}` | `use_joint_feedback: true`, `joint_encoder_joints: [all]`, `encoder_serial_port`/`_baudrate` | `OrcaHandJointFeedback` |
| `orcahand-full-{side}` | both of the above | `OrcaHandFull` |

Left vs. right configs differ only in `type` and the sign/value of every `joint_to_motor_map` entry
(motor IDs are mirrored per side) — ROMs, joint IDs, and the calibration sequence are otherwise
identical.

---

## 12. Public API surface

Everything importable as `orca_core.X` — 27 names, pinned byte-for-byte by
`tests/test_public_api_surface.py`'s `PACKAGE_EXPORTS`:

`BaseHand` · `BaseHandConfig`, `HandConfigValidationError`, `OrcaHandConfig`, `OrcaHandTouchConfig`,
`canonical_joint_ids` · `CalibrationResult` · `OrcaHand`, `MockOrcaHand` · `OrcaHandTouch`,
`OrcaHandJointFeedback`, `OrcaHandFull`, `MockOrcaHandTouch`, `MockOrcaHandJointFeedback`,
`MockOrcaHandFull`, `JointFeedbackConnectError` · `load_hand`, `detect_hand`, `HandDetection` ·
`EncodersNotAvailableError` · `LinkHealth`, `TaxelData` · `OrcaJointPositions` · `HandKinematics`,
`Transform`, `frames` · `LATEST_VERSION`

**Deliberately not exported** at the top level (import from the submodule directly):
`demo_poses.load_demo_poses` ("demo content is not part of the hand-control API"),
`orca_core.constants.*`, `orca_core.api.api.app`, the `HandConfig` backward-compat alias,
`JointEncoderCal`.

The same test file also pins each hand class's *public method set* — `BaseHand`, `OrcaHand`,
`OrcaHandTouch`, `OrcaHandJointFeedback` each get their own frozen name set, and `OrcaHandFull` is
asserted to add nothing beyond their union (§3). Per the superproject `CLAUDE.md`: changing any of
this is a breaking change requiring a deliberate minor-version bump — and `release.yml` publishes to
PyPI on every push to `main` carrying a new version, so this isn't a hypothetical concern.

---

## 13. Scripts (`scripts/`)

All thin CLI front-ends: argparse + print + `input()`, no logic of their own. Most take an optional
positional `config_path` (defaults to the bundled model) and a `--mock` flag via `utils/cli.py`.

| Script | Purpose | Key flags | API used |
|---|---|---|---|
| `calibrate.py` | Run the hardstop-drive calibration sequence | `--force-wrist`, `--fingers`, `--joints`, `--encoder-port`, `--mock` | `OrcaHand`/`MockOrcaHand` directly (bypasses `load_hand()` so a second reader can't corrupt the encoder stream) |
| `tension.py` | Wind tendons taut, then hold motors for manual tensioning | `--move-motors`/`--no-move-motors` | `utils/cli.py` helpers, `hand.tension()` |
| `neutral.py` | Init joints (calibrate if needed), move to neutral | `--force-calibrate` | `utils/cli.py`, `hand.init_joints()` + `set_neutral_position()` |
| `zero.py` | Move every joint to angle 0 | `--force-calibrate`, `--num-steps`, `--step-size` | `utils/cli.py`, `hand.set_zero_position()` |
| `manual_control.py` | Tkinter slider GUI for manual posing; `--motor-space` for tendon bring-up (motor bus only, no encoders) | `--motor-space`, `--encoder-port`, `--max-current`, `--Kp`/`--Ki`, `--fingers`/`--joints` | `load_hand()` (joint-space) or `OrcaHand` directly (`--motor-space`) |
| `check_sensors.py` | Automated pass/fail health check for encoders + tactile | `--port` (bypass config for bare board bring-up), `--encoder-duration` | `load_hand(engage_feedback=False)`, `hardware/sensing/health.py` |
| `monitor_sensors.py` | Live Tkinter view of encoder/tactile streams (no verdicts, just data) | `--port`, `--baud`, `--start-mode` | `HandSerialLink`/`TactileClient`/`JointEncoderClient` directly, no `config_path` at all |
| `configure_motor_chain.py` | First-time assembly: assign motor IDs/baud one at a time | `--reset`, `--baudrate`, `--motor-type` | `maintenance/motor_chain.py` functions directly |
| `setup.py` | Guided end-to-end assembly: tension → calibrate → neutral, with a motion test | — (no `--mock`, always real hardware) | `utils/cli.py`, `hand.tension`/`calibrate`/`set_neutral_position` |
| `stress_test.py` | Endurance/thermal cycling test with live per-joint temperature table | `--num-steps`, `--step-size`, `--hold` | `utils/cli.py`, `hand.get_motor_temp` |
| `check_motor.py` | Standalone single-motor bench check — the one script that's **not** generic | `--port`, `--baudrate`, `--motor_id`, `--wrist`, `--reverse` | **Hardcoded** `DynamixelClient`, not `OrcaHand` at all |

`check_motor.py` is the sole exception to the "generic across motor family" story — it hardcodes
Dynamixel and doesn't go through any config. Everything else is backend-agnostic by construction.

`configure_motor_chain.py`, `tension.py`, `calibrate.py`, and `neutral.py` are assembly/bring-up
scripts whose behavior is mostly in their `maintenance/*.py` backends, not the script itself — see
§9 for the full per-stage walkthrough, including exactly which `control_mode`/current is active at
each step and how that plays out on Feetech vs. Dynamixel hardware.

---

## 14. Examples (`examples/`)

- `demo_runner.py` — library function `run_demo(hand, demo_name, cycles, ...)`: loads
  `demo_poses.load_demo_poses()`, converts each pose's ROM fractions to joint angles, plays the named
  demo's sequence in a loop, returning to neutral between cycles.
- `main_demo.py` / `main_demo_abduction.py` — runnable wrappers around `run_demo` for the `"main"`
  (`open_hand → power_grasp → pinch → neutral`) and `"abduction"` (`fan_out → fan_in →
  spread_grasp → neutral`) demos.
- `record_angles.py` / `record_continuous.py` — kinesthetic-teaching recorders: capture discrete
  waypoints on each Enter press, or continuous samples at a fixed frequency, writing a timestamped
  YAML with metadata (`joint_ids`, `hand_type`).
- `replay_angles.py` / `replay_continuous.py` — play a recorded YAML back, interpolating between
  waypoints (linear or ease-in-out) or pacing continuous frames to real time; both refuse to run if
  the file's `joint_ids` don't match the connected hand.
- `taxel_frames.py` — demonstrates the frame-aware tactile API: streams per-taxel 3D positions +
  force vectors in a chosen frame (`sensor`/`fingertip`/`palm`/`base`/`world`), transformed through
  forward kinematics for joint-dependent frames.
- `sequences/kapandji_opposition.yaml` — a checked-in sample discrete-waypoint recording (v1 joint
  naming) implementing the classic thumb-opposition test; demonstrates `replay_angles.py`'s
  joint-order guard rail (it'll refuse to run against a v2 config).

---

## 15. Tools (`tools/`, maintainer-only)

- `check_downstream.py` — checks whether this working tree is still consistent with sibling repos
  that consume it (`orca_ui`, `orca_teleop`, `orca_firmware`, `orca_stress_tests`, `orca_ros`), since
  those repos' code and tests are invisible to `orca_core`'s own suite. Default mode AST-parses every
  sibling `.py` file for `orca_core` imports and tries to resolve them against the current tree.
  `--run-tests` additionally runs each sibling's own pytest suite in an ephemeral overlay.
  `--symbol NAME` greps every sibling for a reference — **the tool to run before deleting any public
  symbol**, since a green suite (even a green downstream suite) doesn't prove a symbol is dead.
  `--repos-root PATH` overrides where sibling checkouts are looked for.
- `extract_urdf_kinematics.py` — regenerates `kinematics/data/*_kinematics.yaml` from an
  `orcahand_description` URDF file, deriving each joint's sign by cross-checking URDF limits against
  the hand config's ROMs. Run only when the mechanical design changes.

---

## 16. Tests (`tests/`)

Non-test support: `conftest.py` (shared fixtures — `mock_hand`/`connected_mock_hand`/
`initialized_mock_hand` at increasing setup levels, `tactile_mock`/`tactile_mock_factory`,
`encoder_link_and_client`, an autouse `_no_settle_sleeps` that no-ops hardware-pacing waits so the
mock-backed suite runs fast), plus `_encoder_helpers.py`/`_hand_feedback_helpers.py`/`_helpers.py`/
`_loop_helpers.py` (deliberately not `conftest.py`, since pytest loads that as a plugin rather than
an importable module).

| Test file | Covers |
|---|---|
| `test_api.py` | Every FastAPI route against a `MockOrcaHand`: connect/disconnect, torque, joints, calibration, 409 concurrency behavior |
| `test_base_hand_contract.py` | `BaseHand` joint-command coercion, ROM clipping, `pose_from_fractions`, named-position playback |
| `test_calibration.py` | `CalibrationResult` YAML round-tripping, encoder decode, `run_calibration` persistence incl. partial/aborted runs |
| `test_calibration_reference.py` | Golden-file regression: calibration output diffed against a checked-in reference |
| `test_calibration_routine.py` | Failure-path internals: guarded motor reads, offset-cal/torque-release failure handling, atomic YAML write |
| `test_connect_resolution.py` | `connect()`'s motor-driver auto-detection and config persistence |
| `test_core.py` | Import/instantiation smoke tests, an end-to-end mock-hand workflow |
| `test_demo_poses.py` | `load_demo_poses()` validity, fraction bounds, malformed-data errors |
| `test_dynamixel_client_locking.py` | Bus serialization/RX-flushing against a fake `dynamixel_sdk` |
| `test_encoder_anchor_sampling.py` | Anchor-count averaging incl. wraparound, timeout, flagged-sample rejection |
| `test_encoder_polarity_by_side.py` | Pins right-hand polarity table; unvalidated sides fail loudly |
| `test_encoder_protocol.py` | Pure encoder-frame codec: parity, round-trip, wraparound |
| `test_feetech_client.py` | `FeetechClient` bus behavior: read caching, `last_read_ok`, RX flushing, finite torque/mode retries, bus-lock-per-transaction, `calibrate_offset`. Its `FakePacketHandler` test double must implement `unLockEprom`/`LockEprom` (see §7) or any test that reaches an EEPROM-write code path raises `AttributeError` |
| `test_hand_class_layout.py` | Pins the hand-class MRO/diamond layout (§3) |
| `test_hand_config_loading.py` | Config-loading edge cases (empty file, `sensors.baudrate: auto`) |
| `test_hand_detection.py` | `detect_hand()`'s identity-line parsing and detection ladder |
| `test_hand_factory.py` | `load_hand()`'s class-selection logic, `engage_feedback` override |
| `test_hand_serial_link.py` | `HandSerialLink` frame dispatch, handler round-trip, corruption recovery |
| `test_hardware_constants.py` | Pins literal hardware-wiring tables that can't be derived (polarity, slot maps) |
| `test_hardware_hand.py` | Core `OrcaHand` lifecycle: connect state, control mode, unit conversion, idempotency |
| `test_hardware_hand_full.py` | `OrcaHandFull` shared-link orchestration, rollback behavior |
| `test_hardware_hand_joint_feedback.py` | `OrcaHandJointFeedback` connect/calibration-gate/loop-routing |
| `test_jitter.py` | `hand.jitter(...)` amplitude/frequency bounds, concurrency rejection |
| `test_joint_controller.py` | `JointController` PI math, anti-windup, thread-safety |
| `test_joint_encoder_client.py` | `JointEncoderClient` stream lifecycle against a mock link |
| `test_joint_loop.py` | `JointLoopThread`: bumpless start, watchdog tiers, jitter e-stop |
| `test_kinematics.py` | `Transform` math, kinematic-chain data, left/right mirror symmetry |
| `test_kinematics_extractor.py` | `tools/extract_urdf_kinematics.py`'s sign-resolution logic |
| `test_mock_sensing_connect.py` | Mock sensing hands connect out-of-the-box with no real hardware |
| `test_model_dispatch.py` | Version/model-name dispatch across `get_model_path`/config classes |
| `test_motor_chain.py` | Family-agnostic ID/baud planning against a faked in-memory bus |
| `test_motor_client_contract.py` | Cross-family parity — see callout below |
| `test_motor_client_registry.py` | Same `OPEN_CLIENTS` lifecycle contract, against real client classes |
| `test_public_api_surface.py` | Package export + method-surface pinning — see §12 |
| `test_sensing_errors.py` | Encoder-stream-timeout error contract, export/import-path stability |
| `test_sensing_health.py` | `health.py`'s pass/fail verdict logic on synthetic data |
| `test_serial_discovery.py` | Port auto-discovery, VID detection, baud fallback |
| `test_tactile_protocol.py` | Pure tactile wire-protocol codec |
| `test_tactile_rearm.py` | The tactile stream's self-healing re-arm behavior |
| `test_tactile_sensor.py` | `TactileClient` integration against a mock link |
| `test_taxel_frames.py` | Frame-aware taxel API on `OrcaHandTouch` |
| `test_taxel_geometry.py` | Static per-taxel geometry loading |
| `test_tension.py` | `hand.tension(...)` phase events, interrupt handling |
| `test_touch_connect.py` | `OrcaHandTouch` connect/disconnect contract, port fallback |
| `test_yaml.py` | `utils.utils` YAML helpers |

**`test_public_api_surface.py`** pins the entire public namespace *and* every hand class's public
method set exactly — see §12; this is the enforced breaking-change tripwire.

**`test_motor_client_contract.py`** enforces four parity guarantees across `DynamixelClient`/
`FeetechClient`/`MockDynamixelClient`: every bus method raises the same `OSError` when called
disconnected; unknown motor IDs are reported as failed, never exceptioned; the `OPEN_CLIENTS`
registry only gains an entry on a *successful* connect and always loses it on disconnect (even if
torque-off raises); and `set_torque_enabled`'s retry defaults (`retries=3`, `retry_interval=0.25`)
are identical across all three classes via `inspect.signature` — "a dead motor can never wedge the
hand," regardless of motor family.

---

## 17. Cross-cutting conventions

- **Joint naming:** `{finger}_{joint_type}`, except `wrist` (no suffix). Fingers: `thumb`, `index`,
  `middle`, `ring`, `pinky`. Joint types: `cmc`, `mcp`, `pip`, `dip`, `abd` (abduction).
- **Torque is never enabled implicitly**, in any layer — on real hardware, torque-enable,
  maintenance operations, and current-limit raises are always explicitly gated; de-escalation
  (e-stop, torque off, park, stop) never is.
- **Progress/prompt callbacks, not `print`/`input`** — see §9. This is what lets `orca_ui` (or any
  future front-end) drive the identical maintenance code a terminal script runs.
- **`config.yaml` is the source of truth for a hand** — §11; `load_hand()` picks the hand class from
  it, and any external front-end should adapt to whatever it reports, not hardcode assumptions.
- **v1/v2 model versioning** — §11; a version bump usually touches several sibling repos too
  (`orcahand_description`, `orca_sim`, `orcahand_hardware`), per the superproject `CLAUDE.md`.
- **`docs/` is partially stale — verify against code.** Concretely confirmed: `docs/pages/orca-core-docs/orca-core-scripts.md`
  documents `calibrate.py` as taking only `config_path`, but the real script also has
  `--force-wrist`/`--fingers`/`--joints`/`--encoder-port`/`--mock`. Two pages under
  `docs/pages/orca-core-docs/` (`latency-optimization.md`, `motor-client-api.md`) aren't even wired
  into the site's nav. Trust this document and the code over `docs/`; if in doubt, read the source.
- **Downstream repos are invisible to this repo's test suite** — a green `orca_core` suite (even a
  green downstream suite) doesn't prove a public symbol is unused. Run `tools/check_downstream.py`
  first (§15).

---

## 18. Getting started

```bash
cd orca_core
uv sync --group dev            # creates .venv; use uv for everything here, never system Python
uv run pytest tests/           # full suite
uv run pytest tests/test_foo.py::test_bar -x   # single test

# Real-hardware bring-up order, once wired to a hand:
uv run python scripts/tension.py orca_core/models/v2/orcahand-right/config.yaml
uv run python scripts/calibrate.py orca_core/models/v2/orcahand-right/config.yaml
uv run python scripts/neutral.py orca_core/models/v2/orcahand-right/config.yaml

# Or the guided end-to-end path:
uv run python scripts/setup.py orca_core/models/v2/orcahand-right/config.yaml
```

No hardware needed to explore the code: `--mock` on nearly every script builds a `MockOrcaHand`
instead (note: mocks always simulate the Dynamixel family regardless of `config.yaml`'s
`motor_type`, so they won't exercise Feetech-specific behavior).

If you're touching the public API surface (§12) or deleting anything that looks unused, read
`orca_core/CLAUDE.md`'s "Downstream consumers" section and run `tools/check_downstream.py` before
concluding it's safe.

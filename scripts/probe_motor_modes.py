# ==============================================================================
# Copyright (c) 2025 ORCA
#
# This file is part of ORCA and is licensed under the MIT License.
# You may use, copy, modify, and distribute this file under the terms of the MIT License.
# See the LICENSE file at the root of this repository for full license information.
# ==============================================================================
"""Empirical control_mode / current-limit probe for a connected hand's motors.

Applies a chosen ``control_mode`` and current limit, then streams live
position/current telemetry while you physically try to block or move the
motor by hand -- so "does the current limit actually cap stall force on this
motor family" can be answered by measurement instead of documentation.

Works through OrcaHand's public API only (connect, enable_torque,
set_control_mode, set_max_current, get_motor_pos, get_motor_current,
write_motor_pos), in raw motor-space, the same way
``manual_control.py --motor-space`` does -- no calibration required.

Two subcommands:

  probe   Apply one control_mode + current to --motor-ids, hold/nudge them,
          and stream telemetry (including a running max-current-seen stat)
          until Ctrl+C.

  stages  Guided replay of the real lifecycle stages (tension winding/
          holding, calibration drive, neutral move, normal operation),
          reading control_mode/current from config.yaml -- the same values
          tension.py/calibrate.py/init_joints() actually use -- so results
          generalize to those scripts.

The subcommand comes first on the command line (config_path/--motor-ids/etc.
belong to the subcommand, not the top-level parser):

    uv run python scripts/probe_motor_modes.py probe <config.yaml> --motor-ids 3 \\
        --mode current_based_position --current 150

    uv run python scripts/probe_motor_modes.py stages <config.yaml> --motor-ids 3

Note: --mock always simulates a Dynamixel-shaped mock regardless of
config.yaml's motor_type (every MockOrcaHand does) -- useful only as a
structural smoke test of this script, never for validating real Feetech
current behavior.

Note: set_max_current is hand-wide in OrcaHand's own API -- it always applies
to every motor in config.yaml, not just --motor-ids. This script surfaces
that explicitly rather than pretending otherwise.

Bench-testing a single motor before the rest of the hand is assembled? Pass
--isolate: it trims config.yaml down to exactly --motor-ids (motor_ids,
joint_ids, joint_to_motor_map, ROMs, calibration_sequence) before connecting,
so connect()/get_motor_pos()/get_motor_current()/set_max_current() -- all
hand-wide over config.motor_ids -- never try to reach a motor that isn't
physically on the bus yet. Without --isolate, the full hand's motor_ids are
used as configured (correct when the whole hand really is connected).
"""

from __future__ import annotations

import argparse
import dataclasses
import time
from collections import deque

import numpy as np

from orca_core import MockOrcaHand, OrcaHand
from orca_core.constants import CONTROL_MODES
from orca_core.utils.cli import add_hand_arguments, connect_hand, shutdown_hand

STALL_WINDOW_S = 1.0
STALL_THRESHOLD_RAD = 0.01


class MotorWatcher:
    """Tracks per-motor position history for a MOVING/STALLED verdict, and the
    maximum |current| observed since it was created."""

    def __init__(self, motor_ids: list[int]):
        self.motor_ids = motor_ids
        self._history: dict[int, deque[tuple[float, float]]] = {
            mid: deque() for mid in motor_ids
        }
        self.max_current: dict[int, float] = {mid: 0.0 for mid in motor_ids}

    def update(self, t: float, positions: dict[int, float], currents: dict[int, float]) -> dict[int, str]:
        status = {}
        for mid in self.motor_ids:
            hist = self._history[mid]
            hist.append((t, positions[mid]))
            while hist and t - hist[0][0] > STALL_WINDOW_S:
                hist.popleft()
            span = max(p for _, p in hist) - min(p for _, p in hist)
            status[mid] = "STALLED" if span < STALL_THRESHOLD_RAD else "MOVING"
            self.max_current[mid] = max(self.max_current[mid], abs(currents[mid]))
        return status


def _print_header(hand: OrcaHand) -> None:
    print(
        f"motor_type={hand.config.motor_type!r} port={hand.config.port!r} "
        f"control_mode(config default)={hand.config.control_mode!r}"
    )


def _confirm_torque(motor_ids: list[int]) -> None:
    input(
        f"About to enable torque on motor(s) {motor_ids}. "
        "Press Enter to continue, Ctrl+C to abort..."
    )


def _apply_mode(hand: OrcaHand, motor_ids: list[int], mode: str, current: float | None) -> None:
    hand.set_control_mode(mode, motor_ids)
    if current is not None:
        print(
            f"set_max_current({current}) -- note: this applies hand-wide to every "
            "motor in config.yaml, not just --motor-ids (OrcaHand's own API has no "
            "per-call subset for this)."
        )
        hand.set_max_current(current)


def _command_target(hand: OrcaHand, motor_ids: list[int], offset_rad: float) -> None:
    positions = hand.get_motor_pos(as_dict=True)
    targets = np.array([positions[mid] + offset_rad for mid in motor_ids], dtype=float)
    hand.write_motor_pos(motor_ids, targets)


def _stream_telemetry(
    hand: OrcaHand,
    motor_ids: list[int],
    watcher: MotorWatcher,
    *,
    duration: float | None,
    rate: float,
) -> None:
    """Prints live position/current telemetry until Ctrl+C or `duration` elapses."""
    period = 1.0 / rate
    start = time.monotonic()
    try:
        while duration is None or time.monotonic() - start < duration:
            t = time.monotonic() - start
            positions = hand.get_motor_pos(as_dict=True)
            currents = hand.get_motor_current(as_dict=True)
            status = _safe_update(watcher, t, positions, currents)
            line = " | ".join(
                f"motor {mid}: pos={positions[mid]:+.3f}rad "
                f"current={currents[mid]:+.1f}mA "
                f"(max seen {watcher.max_current[mid]:.1f}mA) {status[mid]}"
                for mid in motor_ids
            )
            print(f"[t={t:5.1f}s] {line}")
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n(stopped)")


def _safe_update(watcher: MotorWatcher, t, positions, currents):
    return watcher.update(t, positions, currents)


def cmd_probe(hand: OrcaHand, args: argparse.Namespace) -> None:
    motor_ids = args.motor_ids
    _print_header(hand)
    print(f"Starting position: {hand.get_motor_pos(as_dict=True)}")

    _confirm_torque(motor_ids)
    failed = hand.enable_torque(motor_ids)
    if failed:
        print(f"WARNING: motors {failed} did not acknowledge torque enable")

    _apply_mode(hand, motor_ids, args.mode, args.current)
    offset = 0.0 if args.hold else args.nudge_rad
    _command_target(hand, motor_ids, offset)

    watcher = MotorWatcher(motor_ids)
    print(
        f"mode={args.mode!r} current={args.current}mA -- physically try to block "
        "the motor by hand now. Ctrl+C to stop."
    )
    _stream_telemetry(hand, motor_ids, watcher, duration=None, rate=args.rate)

    print(
        "Max current seen: "
        + ", ".join(f"motor {mid}: {watcher.max_current[mid]:.1f}mA" for mid in motor_ids)
    )


# (stage name, mode override or None to use config.control_mode, current selector)
# current selector is a callable(config, wrist_motor_id, motor_ids) -> float | None
def _tension_winding_current(cfg, _wrist_motor_id, _motor_ids):
    return cfg.calibration_current


def _tension_holding_current(cfg, _wrist_motor_id, _motor_ids):
    return cfg.max_current


def _calibration_current(cfg, wrist_motor_id, motor_ids):
    if wrist_motor_id is not None and wrist_motor_id in motor_ids:
        return cfg.wrist_calibration_current
    return cfg.calibration_current


def _no_current(_cfg, _wrist_motor_id, _motor_ids):
    return None  # neutral-move: no set_max_current call at all, matches init_joints()


def _normal_operation_current(cfg, _wrist_motor_id, _motor_ids):
    return cfg.max_current


STAGES = [
    ("tension-winding", "current_based_position", _tension_winding_current),
    ("tension-holding", "current_based_position", _tension_holding_current),
    ("calibration-drive", "current_based_position", _calibration_current),
    ("neutral-move", "position", _no_current),
    ("normal-operation", None, _normal_operation_current),  # None -> cfg.control_mode
]


def cmd_stages(hand: OrcaHand, args: argparse.Namespace) -> None:
    motor_ids = args.motor_ids
    cfg = hand.config
    wrist_motor_id = cfg.joint_to_motor_map.get("wrist")

    _print_header(hand)
    _confirm_torque(motor_ids)
    failed = hand.enable_torque(motor_ids)
    if failed:
        print(f"WARNING: motors {failed} did not acknowledge torque enable")

    summary: dict[str, dict[int, float]] = {}
    for name, mode_override, current_fn in STAGES:
        mode = mode_override or cfg.control_mode
        current = current_fn(cfg, wrist_motor_id, motor_ids)

        input(
            f"\n--- Stage: {name} (mode={mode!r}, current={current}) --- "
            "Press Enter to start (Ctrl+C during telemetry to skip to next)..."
        )
        _apply_mode(hand, motor_ids, mode, current)
        _command_target(hand, motor_ids, 0.0)

        watcher = MotorWatcher(motor_ids)
        _stream_telemetry(hand, motor_ids, watcher, duration=args.duration, rate=args.rate)
        summary[name] = dict(watcher.max_current)
        print(
            f"[{name}] max current seen: "
            + ", ".join(f"motor {mid}: {v:.1f}mA" for mid, v in summary[name].items())
        )

    print("\n=== Summary: max current seen per stage ===")
    for name, currents in summary.items():
        print(f"{name:20s} " + ", ".join(f"motor {mid}: {v:.1f}mA" for mid, v in currents.items()))


def _common_parser() -> argparse.ArgumentParser:
    """Options shared by every subcommand: config_path/--mock (via
    add_hand_arguments) plus --motor-ids/--rate. Added as a parent to each
    subparser -- rather than to the top-level parser -- because a bare
    ``nargs="?"`` positional (config_path) ahead of a subparsers action is an
    argparse ambiguity: it can bind the wrong token to the wrong slot. With
    subcommand-first parsing there's no ambiguity to resolve.
    """
    common = argparse.ArgumentParser(add_help=False)
    add_hand_arguments(common)
    common.add_argument(
        "--motor-ids",
        type=int,
        nargs="+",
        required=True,
        help="Motor IDs to probe. Required -- never defaults to all motors.",
    )
    common.add_argument(
        "--rate", type=float, default=10.0, help="Telemetry print rate in Hz (default 10)."
    )
    common.add_argument(
        "--isolate",
        action="store_true",
        help="Trim config.yaml down to exactly --motor-ids before connecting, so "
        "connect()/get_motor_pos()/get_motor_current()/set_max_current() (all "
        "hand-wide over config.motor_ids) never try to reach a motor that isn't "
        "physically on the bus yet. Use this when bench-testing one or a few "
        "motors before the rest of the hand is assembled.",
    )
    return common


def _scope_config_to_motors(config, motor_ids: list[int]):
    """Returns a copy of *config* trimmed to exactly *motor_ids*.

    ``dataclasses.replace`` re-runs the config's own ``__post_init__``
    validation on the result, so an inconsistent trim (e.g. a motor id with
    no joint) fails loudly here rather than surfacing as a confusing connect()
    error later.
    """
    motor_to_joint = {mid: joint for joint, mid in config.joint_to_motor_map.items()}
    missing = [mid for mid in motor_ids if mid not in motor_to_joint]
    if missing:
        raise ValueError(
            f"motor id(s) {missing} not found in config.yaml's joint_to_motor_map"
        )
    joint_ids = [motor_to_joint[mid] for mid in motor_ids]

    return dataclasses.replace(
        config,
        motor_ids=list(motor_ids),
        joint_ids=joint_ids,
        joint_to_motor_map={j: config.joint_to_motor_map[j] for j in joint_ids},
        joint_inversion_dict={
            j: config.joint_inversion_dict.get(j, False) for j in joint_ids
        },
        joint_roms_dict={j: config.joint_roms_dict[j] for j in joint_ids},
        neutral_position={
            j: config.neutral_position[j] for j in joint_ids if j in config.neutral_position
        },
        # Not needed for probing/stage-replay (this script drives current/mode/
        # position directly) and would otherwise reference joints we just
        # dropped, which validate_config() rejects.
        calibration_sequence=[],
    )


def _load_hand(config_path: str | None, motor_ids: list[int], *, use_mock: bool, isolate: bool):
    hand_cls = MockOrcaHand if use_mock else OrcaHand
    if not isolate:
        return hand_cls(config_path=config_path)

    full_config = hand_cls.config_cls.from_config_path(config_path)
    scoped_config = _scope_config_to_motors(full_config, motor_ids)
    print(
        f"--isolate: connecting with only {scoped_config.joint_ids} "
        f"(motor(s) {scoped_config.motor_ids}) instead of the full hand."
    )
    return hand_cls(config=scoped_config)


def build_parser() -> argparse.ArgumentParser:
    common = _common_parser()

    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = parser.add_subparsers(dest="subcommand", required=True)

    p_probe = sub.add_parser(
        "probe",
        parents=[common],
        help="Apply one control_mode + current and stream telemetry until Ctrl+C.",
    )
    p_probe.add_argument("--mode", choices=CONTROL_MODES, required=True)
    p_probe.add_argument(
        "--current", type=float, required=True, help="Current/torque limit in mA."
    )
    p_probe.add_argument(
        "--nudge-rad",
        type=float,
        default=None,
        help="Command current position + this offset (rad) instead of holding in place.",
    )

    p_stages = sub.add_parser(
        "stages",
        parents=[common],
        help="Guided replay of tension/calibration/neutral/normal-operation, using "
        "config.yaml's real values.",
    )
    p_stages.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Seconds to stream telemetry per stage (default 10).",
    )

    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    if args.subcommand == "probe":
        args.hold = args.nudge_rad is None

    hand = _load_hand(
        args.config_path, args.motor_ids, use_mock=args.mock, isolate=args.isolate
    )
    connect_hand(hand)
    try:
        if args.subcommand == "probe":
            cmd_probe(hand, args)
        elif args.subcommand == "stages":
            cmd_stages(hand, args)
    finally:
        try:
            hand.disable_torque(args.motor_ids)
        except Exception as exc:
            print(f"disable_torque failed during shutdown: {exc}")
        shutdown_hand(hand)


if __name__ == "__main__":
    main()

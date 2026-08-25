#!/usr/bin/env python3
"""Run Humanoid Ultra Isaac Lab policies in MuJoCo.

Without --policy this loads walk, stand, Pick, houtaitui, Spin and Taitui-Left
from the local ``pt`` directory.  Gamepad X (or keyboard P) switches
stand/walk.  LT+D-pad Right/Down/Up/Left starts Pick/houtaitui/Spin/Taitui-Left
and RT+D-pad Right starts Taitui-Right,
respectively.  Every Mimic reference advances once per 50 Hz policy step and
returns directly to walk through the same smooth policy-target blend used by
stand/walk.

Use ``--mode mimic --policy ... --motion-file ...`` for direct single-policy
testing.  Gamepad LT+B (or keyboard 0) stops inference and latches kd-only
damping until reset (R).
"""

from __future__ import annotations

import argparse
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
import torch


SCRIPT_DIR = Path(__file__).resolve().parent
POLICY_DIR = SCRIPT_DIR / "pt"
DEFAULT_STAND_POLICY = POLICY_DIR / "zxh-stand-leftarm" / "policy.pt"
DEFAULT_WALK_POLICY = POLICY_DIR / "zxh-walk" / "policy.pt"
DEFAULT_LEFT_ARM_TRAJ = (
    POLICY_DIR / "zxh-stand-leftarm" / "left_wrist_pitch_traj.csv"
)


ISAAC_12DOF_JOINTS = (
    "left_hip_roll_joint",
    "right_hip_roll_joint",
    "left_hip_yaw_joint",
    "right_hip_yaw_joint",
    "left_hip_pitch_joint",
    "right_hip_pitch_joint",
    "left_knee_joint",
    "right_knee_joint",
    "left_ankle_pitch_joint",
    "right_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_ankle_roll_joint",
)

ISAAC_27DOF_JOINTS = (
    "left_hip_roll_joint",
    "right_hip_roll_joint",
    "waist_yaw_joint",
    "left_hip_yaw_joint",
    "right_hip_yaw_joint",
    "left_shoulder_pitch_joint",
    "right_shoulder_pitch_joint",
    "left_hip_pitch_joint",
    "right_hip_pitch_joint",
    "left_shoulder_roll_joint",
    "right_shoulder_roll_joint",
    "left_knee_joint",
    "right_knee_joint",
    "left_shoulder_yaw_joint",
    "right_shoulder_yaw_joint",
    "left_ankle_pitch_joint",
    "right_ankle_pitch_joint",
    "left_elbow_joint",
    "right_elbow_joint",
    "left_ankle_roll_joint",
    "right_ankle_roll_joint",
    "left_wrist_yaw_joint",
    "right_wrist_yaw_joint",
    "left_wrist_roll_joint",
    "right_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "right_wrist_pitch_joint",
)

# Isaac/PhysX stores articulation bodies breadth-first.  The motion NPZ files
# are written directly from ``robot.data.body_*_w`` in this order.
ISAAC_27DOF_BODIES = (
    "base_link",
    "left_hip_roll_link",
    "right_hip_roll_link",
    "trunk_link",
    "left_hip_yaw_link",
    "right_hip_yaw_link",
    "left_shoulder_pitch_link",
    "right_shoulder_pitch_link",
    "left_hip_pitch_link",
    "right_hip_pitch_link",
    "left_shoulder_roll_link",
    "right_shoulder_roll_link",
    "left_knee_link",
    "right_knee_link",
    "left_shoulder_yaw_link",
    "right_shoulder_yaw_link",
    "left_ankle_pitch_link",
    "right_ankle_pitch_link",
    "left_elbow_link",
    "right_elbow_link",
    "left_ankle_roll_link",
    "right_ankle_roll_link",
    "left_wrist_yaw_link",
    "right_wrist_yaw_link",
    "left_wrist_roll_link",
    "right_wrist_roll_link",
    "left_wrist_pitch_link",
    "right_wrist_pitch_link",
)
MIMIC_ANCHOR_BODY_NAME = "trunk_link"
MIMIC_PROPRIO_HISTORY_LENGTH = 10

LEFT_ARM_JOINTS = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
)
LEFT_ARM_COMMAND_DIM = 2 * len(LEFT_ARM_JOINTS) + 1

URDF_12DOF_JOINTS = (
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_hip_pitch_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_hip_pitch_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
)

URDF_27DOF_JOINTS = URDF_12DOF_JOINTS + (
    "waist_yaw_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_yaw_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_yaw_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
)

DEPLOYMENT_POSITION_LIMITS = {
    "left_hip_roll_joint": (-0.25, 1.5708),
    "right_hip_roll_joint": (-1.5708, 0.25),
    "left_hip_yaw_joint": (-1.5708, 1.5708),
    "right_hip_yaw_joint": (-1.5708, 1.5708),
    "left_hip_pitch_joint": (-1.5708, 1.5708),
    "right_hip_pitch_joint": (-1.5708, 1.5708),
    "left_knee_joint": (0.0, 2.356),
    "right_knee_joint": (0.0, 2.356),
    "left_ankle_pitch_joint": (-0.7, 0.95),
    "right_ankle_pitch_joint": (-0.7, 0.95),
    "left_ankle_roll_joint": (-0.5236, 0.5236),
    "right_ankle_roll_joint": (-0.5236, 0.5236),
    "waist_yaw_joint": (-2.618, 2.618),
    "left_shoulder_pitch_joint": (-2.4, 1.2),
    "right_shoulder_pitch_joint": (-1.2, 2.4),
    "left_shoulder_roll_joint": (-0.3, 2.7),
    "right_shoulder_roll_joint": (-2.7, 0.3),
    "left_shoulder_yaw_joint": (-2.5, 2.5),
    "right_shoulder_yaw_joint": (-2.5, 2.5),
    "left_elbow_joint": (-2.17, 0.0),
    "right_elbow_joint": (0.0, 2.17),
    "left_wrist_yaw_joint": (-2.5, 2.5),
    "right_wrist_yaw_joint": (-2.5, 2.5),
    "left_wrist_roll_joint": (-1.11, 1.11),
    "right_wrist_roll_joint": (-1.11, 1.11),
    "left_wrist_pitch_joint": (-1.05, 1.05),
    "right_wrist_pitch_joint": (-1.05, 1.05),
}


@dataclass(frozen=True)
class PolicySpec:
    name: str
    mode: str  # "stand", "locomotion", or "mimic" observation semantics.
    path: Path
    use_current_asset_defaults: bool = False


@dataclass(frozen=True)
class MimicActionSpec:
    """A named one-shot Mimic policy, reference motion and input bindings."""

    name: str
    trigger: str
    policy_path: Path
    motion_path: Path
    keyboard_key: str = ""
    start_frame: int = 0
    target_speed: float = 6.0
    use_current_asset_defaults: bool = False


DEFAULT_MIMIC_ACTIONS = (
    MimicActionSpec(
        name="pick",
        trigger="right",
        policy_path=POLICY_DIR / "zxh-mimic-pick" / "policy.pt",
        motion_path=(
            POLICY_DIR
            / "zxh-mimic-pick"
            / "ustc1_pick_stand_transition.npz"
        ),
        keyboard_key="m",
        target_speed=6.0,
        use_current_asset_defaults=True,
    ),
    MimicActionSpec(
        name="houtaitui",
        trigger="down",
        policy_path=POLICY_DIR / "zxh-mimic-houtaitui" / "policy.pt",
        motion_path=(
            POLICY_DIR
            / "zxh-mimic-houtaitui"
            / "ustc1_rightstand_stand_transition.npz"
        ),
        keyboard_key="h",
        target_speed=6.0,
        # The 2026-08-24 policy is a current-defaults 144-D one; the 07-24 v3 it
        # replaced was a legacy 144-D policy with the old shoulder-roll zero
        # point, so this flag has to flip with the file.
        use_current_asset_defaults=True,
    ),
    MimicActionSpec(
        name="taitui_right",
        trigger="rt_right",  # LT+D-pad was full; RT is the second modifier layer
        policy_path=POLICY_DIR / "zxh-mimic-taitui-right" / "policy.pt",
        motion_path=(
            POLICY_DIR
            / "zxh-mimic-taitui-right"
            / "ustc_taitui_right_stand_transition.npz"
        ),
        keyboard_key="y",
        target_speed=6.0,
        use_current_asset_defaults=True,
    ),
    MimicActionSpec(
        name="spin",
        trigger="up",
        policy_path=POLICY_DIR / "zxh-mimic-spin" / "policy.pt",
        motion_path=(
            POLICY_DIR
            / "zxh-mimic-spin"
            / "ustc1_spin_stand_transition_hold_2p5s.npz"
        ),
        keyboard_key="i",
        target_speed=6.0,
        use_current_asset_defaults=True,
    ),
    MimicActionSpec(
        name="taitui_left",
        trigger="left",
        policy_path=POLICY_DIR / "zxh-mimic-taitui-left" / "policy.pt",
        motion_path=(
            POLICY_DIR
            / "zxh-mimic-taitui-left"
            / "ustc_taitui_left_stand_transition.npz"
        ),
        keyboard_key="t",
        target_speed=6.0,
        use_current_asset_defaults=True,
    ),
)


@dataclass(frozen=True)
class RobotProfile:
    dof: int
    root_height: float
    joint_names: tuple[str, ...]
    default_joint_pos: np.ndarray
    stiffness: np.ndarray
    damping: np.ndarray
    torque_limits: np.ndarray
    velocity_limits: np.ndarray
    position_limits: np.ndarray

    @property
    def observation_dim(self) -> int:
        return 9 + 3 * self.dof

    @property
    def mimic_observation_dim(self) -> int:
        # 2*dof reference command + 6-D anchor orientation + 3-D base angular
        # velocity + joint position/velocity + previous action.
        return 9 + 5 * self.dof

    @property
    def mimic_history_observation_dim(self) -> int:
        # Current reference command/orientation and last applied action, plus
        # term-major histories of IMU linear/angular motion and joint q/dq.
        return (
            6
            + 3 * self.dof
            + MIMIC_PROPRIO_HISTORY_LENGTH * (6 + 2 * self.dof)
        )

    @property
    def mimic_observation_dims(self) -> tuple[int, int]:
        return self.mimic_observation_dim, self.mimic_history_observation_dim


class GamepadCommandSource:
    """Poll an SDL-mapped gamepad without blocking the MuJoCo control loop."""

    AXIS_MAX = 32768.0
    RECONNECT_PERIOD = 1.0

    # (modifier trigger, D-pad button).  The four LT combinations filled up, so
    # RT is a second layer -- it was otherwise unused.
    SUPPORTED_MIMIC_TRIGGERS = {
        "right": ("TRIGGERLEFT", "CONTROLLER_BUTTON_DPAD_RIGHT"),
        "down": ("TRIGGERLEFT", "CONTROLLER_BUTTON_DPAD_DOWN"),
        "up": ("TRIGGERLEFT", "CONTROLLER_BUTTON_DPAD_UP"),
        "left": ("TRIGGERLEFT", "CONTROLLER_BUTTON_DPAD_LEFT"),
        "rt_right": ("TRIGGERRIGHT", "CONTROLLER_BUTTON_DPAD_RIGHT"),
        "rt_down": ("TRIGGERRIGHT", "CONTROLLER_BUTTON_DPAD_DOWN"),
        "rt_up": ("TRIGGERRIGHT", "CONTROLLER_BUTTON_DPAD_UP"),
        "rt_left": ("TRIGGERRIGHT", "CONTROLLER_BUTTON_DPAD_LEFT"),
    }

    @staticmethod
    def trigger_label(trigger: str) -> str:
        """Human-readable binding, e.g. 'LT+D-pad Down' or 'RT+D-pad Right'."""
        modifier, direction = ("RT", trigger[3:]) if trigger.startswith("rt_") else ("LT", trigger)
        return f"{modifier}+D-pad {direction.title()}"

    def __init__(
        self, index: int, deadzone: float, mimic_triggers: dict[str, str]
    ):
        try:
            import pygame
            from pygame._sdl2 import controller
        except ImportError as exc:
            raise RuntimeError(
                "Gamepad control requires pygame. Install it with `pip install pygame`."
            ) from exc

        self._pygame = pygame
        self._controller_module = controller
        self.index = index
        self.deadzone = deadzone
        self._controller = None
        self._next_connect_attempt = 0.0
        self._waiting_reported = False
        self._previous_y = False
        self._previous_x = False
        self._previous_a = False
        unsupported_triggers = set(mimic_triggers) - set(
            self.SUPPORTED_MIMIC_TRIGGERS
        )
        if unsupported_triggers:
            raise ValueError(
                f"Unsupported Mimic gamepad triggers: {sorted(unsupported_triggers)}"
            )
        self.mimic_triggers = dict(mimic_triggers)
        self._previous_mimic = {
            trigger: False for trigger in self.mimic_triggers
        }
        self._previous_stop = False
        self._previous_damping = False
        self.x_toggle_requested = False
        self.a_toggle_requested = False
        self.mimic_requested: str | None = None
        self.damping_stop_requested = False
        self.enabled = True
        self.connected = False

        self._controller_module.init()
        self._try_connect()

    def _try_connect(self) -> None:
        now = time.monotonic()
        if now < self._next_connect_attempt:
            return
        self._next_connect_attempt = now + self.RECONNECT_PERIOD

        try:
            self._controller_module.update()
            if (
                self.index >= self._controller_module.get_count()
                or not self._controller_module.is_controller(self.index)
            ):
                if not self._waiting_reported:
                    print(
                        f"Gamepad {self.index} is unavailable or has no SDL mapping; "
                        "waiting for a controller."
                    )
                    self._waiting_reported = True
                return
            self._controller = self._controller_module.Controller(self.index)
        except (self._pygame.error, self._controller_module.error) as exc:
            if not self._waiting_reported:
                print(f"Cannot open gamepad {self.index}: {exc}; waiting for a controller.")
                self._waiting_reported = True
            return

        self.connected = True
        self._waiting_reported = False
        self._previous_y = False
        self._previous_x = False
        self._previous_a = False
        self._previous_mimic = {
            trigger: False for trigger in self.mimic_triggers
        }
        self._previous_stop = False
        self._previous_damping = False
        self.x_toggle_requested = False
        self.a_toggle_requested = False
        self.mimic_requested = None
        self.damping_stop_requested = False
        state = "enabled" if self.enabled else "disabled; press Y to enable"
        print(f"Gamepad connected: {self._controller.name} ({state}).")

    def _disconnect(self) -> None:
        if self._controller is not None:
            try:
                self._controller.quit()
            except self._pygame.error:
                pass
        self._controller = None
        self.connected = False
        self.enabled = False
        self._previous_y = False
        self._previous_x = False
        self._previous_a = False
        self._previous_mimic = {
            trigger: False for trigger in self.mimic_triggers
        }
        self._previous_stop = False
        self._previous_damping = False
        self.x_toggle_requested = False
        self.a_toggle_requested = False
        self.mimic_requested = None
        self.damping_stop_requested = False
        self._next_connect_attempt = 0.0
        if not self._waiting_reported:
            print("Gamepad disconnected: command cleared; reconnect and press Y to enable.")
            self._waiting_reported = True

    def _axis(self, axis: int) -> float:
        assert self._controller is not None
        value = float(np.clip(self._controller.get_axis(axis) / self.AXIS_MAX, -1.0, 1.0))
        magnitude = abs(value)
        if magnitude <= self.deadzone:
            return 0.0
        return float(np.copysign((magnitude - self.deadzone) / (1.0 - self.deadzone), value))

    def poll(self, mode: str) -> np.ndarray:
        self.x_toggle_requested = False
        self.a_toggle_requested = False
        self.mimic_requested = None
        self.damping_stop_requested = False
        zero_command = np.zeros(3, dtype=np.float64)
        if self._controller is None:
            self._try_connect()
            if self._controller is None:
                return zero_command

        try:
            self._controller_module.update()
            if not self._controller.attached():
                self._disconnect()
                return zero_command

            y_pressed = bool(self._controller.get_button(self._pygame.CONTROLLER_BUTTON_Y))
            x_pressed = bool(self._controller.get_button(self._pygame.CONTROLLER_BUTTON_X))
            a_pressed = bool(self._controller.get_button(self._pygame.CONTROLLER_BUTTON_A))
            lt_pressed = bool(
                self._controller.get_axis(self._pygame.CONTROLLER_AXIS_TRIGGERLEFT)
                > 0.5 * self.AXIS_MAX
            )
            rt_pressed = bool(
                self._controller.get_axis(self._pygame.CONTROLLER_AXIS_TRIGGERRIGHT)
                > 0.5 * self.AXIS_MAX
            )
            modifier_pressed = {"TRIGGERLEFT": lt_pressed, "TRIGGERRIGHT": rt_pressed}
            mimic_pressed = {}
            for trigger, (modifier, button_name) in self.SUPPORTED_MIMIC_TRIGGERS.items():
                if trigger not in self.mimic_triggers:
                    continue
                button = getattr(self._pygame, button_name)
                mimic_pressed[trigger] = bool(
                    modifier_pressed[modifier] and self._controller.get_button(button)
                )
            stop_pressed = bool(
                self._controller.get_button(self._pygame.CONTROLLER_BUTTON_LEFTSHOULDER)
                and self._controller.get_button(self._pygame.CONTROLLER_BUTTON_RIGHTSHOULDER)
            )
            damping_pressed = bool(
                lt_pressed and self._controller.get_button(self._pygame.CONTROLLER_BUTTON_B)
            )
            if damping_pressed and not self._previous_damping:
                self.damping_stop_requested = True
                self.enabled = False
                print("Gamepad LT+B: damping stop requested; gamepad control disabled.")
            if stop_pressed and not self._previous_stop:
                self.enabled = False
                print("Gamepad LB+RB: command cleared and gamepad control disabled.")
            elif y_pressed and not self._previous_y:
                self.enabled = not self.enabled
                print(f"Gamepad control: {'ON' if self.enabled else 'OFF (command cleared)'}.")
            if x_pressed and not self._previous_x and not stop_pressed:
                self.x_toggle_requested = True
            if a_pressed and not self._previous_a and not damping_pressed:
                self.a_toggle_requested = True
            if not damping_pressed and not stop_pressed and self.enabled:
                for trigger, pressed in mimic_pressed.items():
                    if pressed and not self._previous_mimic[trigger]:
                        self.mimic_requested = self.mimic_triggers[trigger]
                        break
            self._previous_y = y_pressed
            self._previous_x = x_pressed
            self._previous_a = a_pressed
            self._previous_mimic = mimic_pressed
            self._previous_stop = stop_pressed
            self._previous_damping = damping_pressed

            if not self.enabled:
                return zero_command

            left_x = self._axis(self._pygame.CONTROLLER_AXIS_LEFTX)
            left_y = self._axis(self._pygame.CONTROLLER_AXIS_LEFTY)
            right_x = self._axis(self._pygame.CONTROLLER_AXIS_RIGHTX)
        except (self._pygame.error, self._controller_module.error):
            self._disconnect()
            return zero_command

        if mode == "stand":
            return np.asarray((-left_y, -0.5 * left_x, 0.5 * right_x), dtype=np.float64)

        forward = -left_y
        vx = forward * (1.0 if forward >= 0.0 else 0.6)
        return np.asarray((vx, -0.5 * left_x, 1.57 * right_x), dtype=np.float64)

    def close(self) -> None:
        if self._controller is not None:
            self._controller.quit()
            self._controller = None
        self._controller_module.quit()


def _gain_for_joint(name: str) -> tuple[float, float, float]:
    if "hip_roll" in name:
        return 150.0, 2.5, 300.0
    if "hip_yaw" in name:
        return 80.0, 0.8, 90.0
    if "hip_pitch" in name or "knee" in name:
        return 180.0, 2.4, 300.0
    if "ankle_pitch" in name:
        return 40.0, 0.8, 27.0
    if "ankle_roll" in name:
        return 20.0, 0.4, 27.0
    if "waist" in name:
        return 150.0, 2.5, 150.0
    if "shoulder" in name:
        return 80.0, 1.5, 60.0
    if "elbow" in name:
        return 60.0, 1.2, 60.0
    if "wrist" in name:
        return 25.0, 0.8, 24.0
    raise ValueError(f"No actuator gains configured for joint: {name}")


def _velocity_limit_for_joint(name: str) -> float:
    if "ankle" in name:
        return 12.0
    if "waist" in name:
        return 12.56
    if any(part in name for part in ("shoulder", "elbow", "wrist")):
        return 10.0
    return 15.0


def make_profile(dof: int) -> RobotProfile:
    if dof == 12:
        names = ISAAC_12DOF_JOINTS
        root_height = 0.995
        leg_pitch = 0.346431
        knee = 0.755514
        ankle_pitch = 0.366252
    elif dof == 27:
        names = ISAAC_27DOF_JOINTS
        root_height = 1.005
        leg_pitch = 0.289936
        knee = 0.742326
        ankle_pitch = 0.409573
    else:
        raise ValueError(f"Unsupported number of joints: {dof}")

    default_by_name = {
        "left_hip_roll_joint": 0.0,
        "right_hip_roll_joint": 0.0,
        "left_hip_yaw_joint": 0.0,
        "right_hip_yaw_joint": 0.0,
        "left_hip_pitch_joint": leg_pitch,
        "right_hip_pitch_joint": leg_pitch,
        "left_knee_joint": knee,
        "right_knee_joint": knee,
        "left_ankle_pitch_joint": ankle_pitch,
        "right_ankle_pitch_joint": ankle_pitch,
        "left_ankle_roll_joint": 0.0,
        "right_ankle_roll_joint": 0.0,
        "waist_yaw_joint": 0.0,
        "left_shoulder_pitch_joint": 0.25,
        "right_shoulder_pitch_joint": -0.25,
        "left_shoulder_roll_joint": -0.05,
        "right_shoulder_roll_joint": 0.05,
        "left_shoulder_yaw_joint": -1.5707963,
        "right_shoulder_yaw_joint": 1.5707963,
        "left_elbow_joint": -0.6,
        "right_elbow_joint": 0.6,
        "left_wrist_yaw_joint": 1.5707963,
        "right_wrist_yaw_joint": -1.5707963,
        "left_wrist_roll_joint": 0.0,
        "right_wrist_roll_joint": 0.0,
        "left_wrist_pitch_joint": 0.0,
        "right_wrist_pitch_joint": 0.0,
    }
    gains = [_gain_for_joint(name) for name in names]
    stiffness, damping, torque_limits = zip(*gains)
    return RobotProfile(
        dof=dof,
        root_height=root_height,
        joint_names=tuple(names),
        default_joint_pos=np.asarray([default_by_name[name] for name in names], dtype=np.float64),
        stiffness=np.asarray(stiffness, dtype=np.float64),
        damping=np.asarray(damping, dtype=np.float64),
        torque_limits=np.asarray(torque_limits, dtype=np.float64),
        velocity_limits=np.asarray(
            [_velocity_limit_for_joint(name) for name in names], dtype=np.float64
        ),
        position_limits=np.asarray(
            [DEPLOYMENT_POSITION_LIMITS[name] for name in names], dtype=np.float64
        ),
    )


def quaternion_to_rotation_matrix(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion / np.linalg.norm(quaternion)
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def quaternion_conjugate(quaternion: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion, dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm < 1.0e-12:
        raise ValueError("Cannot conjugate a zero-length quaternion.")
    result = quaternion / norm
    result[1:] *= -1.0
    return result


def quaternion_multiply(lhs: np.ndarray, rhs: np.ndarray) -> np.ndarray:
    """Multiply two wxyz quaternions."""
    w1, x1, y1, z1 = np.asarray(lhs, dtype=np.float64)
    w2, x2, y2, z2 = np.asarray(rhs, dtype=np.float64)
    result = np.asarray(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )
    norm = float(np.linalg.norm(result))
    if norm < 1.0e-12:
        raise ValueError("Quaternion multiplication produced a zero-length quaternion.")
    return result / norm


def policy_input_dim(policy: torch.jit.ScriptModule) -> int:
    """Read the actor input width from the exported TorchScript policy."""
    state_dict = policy.state_dict()
    first_layer = state_dict.get("actor.0.weight")
    if first_layer is None or first_layer.ndim != 2:
        raise RuntimeError(
            "Cannot determine policy input size: actor.0.weight is missing from policy.pt."
        )
    return int(first_layer.shape[1])


class MimicMotion:
    """50 Hz reference stream matching the Isaac Lab MotionCommand policy input."""

    REQUIRED_KEYS = (
        "fps",
        "joint_pos",
        "joint_vel",
        "body_pos_w",
        "body_quat_w",
        "body_lin_vel_w",
        "body_ang_vel_w",
    )

    def __init__(self, motion_path: Path, profile: RobotProfile, start_frame: int):
        if profile.dof != 27:
            raise ValueError("Mimic motion playback currently requires the 27-DOF robot.")
        if not motion_path.is_file():
            raise FileNotFoundError(f"Mimic motion NPZ not found: {motion_path}")

        with np.load(motion_path) as data:
            missing = [key for key in self.REQUIRED_KEYS if key not in data]
            if missing:
                raise ValueError(f"Mimic motion NPZ is missing keys: {missing}")
            self.fps = float(np.asarray(data["fps"]).reshape(-1)[0])
            self.joint_pos = np.asarray(data["joint_pos"], dtype=np.float64)
            self.joint_vel = np.asarray(data["joint_vel"], dtype=np.float64)
            self.body_pos_w = np.asarray(data["body_pos_w"], dtype=np.float64)
            self.body_quat_w = np.asarray(data["body_quat_w"], dtype=np.float64)
            self.body_lin_vel_w = np.asarray(data["body_lin_vel_w"], dtype=np.float64)
            self.body_ang_vel_w = np.asarray(data["body_ang_vel_w"], dtype=np.float64)

        if not np.isclose(self.fps, 50.0):
            raise ValueError(
                f"Mimic motion must be sampled at 50 Hz, but {motion_path} reports {self.fps:g} Hz."
            )
        expected_joint_shape = (self.joint_pos.shape[0], profile.dof)
        if self.joint_pos.ndim != 2 or self.joint_pos.shape[1] != profile.dof:
            raise ValueError(
                f"Mimic joint_pos must have shape [frames, {profile.dof}], got {self.joint_pos.shape}."
            )
        if self.joint_vel.shape != expected_joint_shape:
            raise ValueError(
                f"Mimic joint_vel must have shape {expected_joint_shape}, got {self.joint_vel.shape}."
            )
        expected_body_shape = (self.joint_pos.shape[0], len(ISAAC_27DOF_BODIES))
        for name, array, width in (
            ("body_pos_w", self.body_pos_w, 3),
            ("body_quat_w", self.body_quat_w, 4),
            ("body_lin_vel_w", self.body_lin_vel_w, 3),
            ("body_ang_vel_w", self.body_ang_vel_w, 3),
        ):
            if array.shape != (*expected_body_shape, width):
                raise ValueError(
                    f"Mimic {name} must have shape {(*expected_body_shape, width)}, got {array.shape}."
                )
        if not 0 <= start_frame < self.joint_pos.shape[0]:
            raise ValueError(
                f"--mimic-start-frame must be in [0, {self.joint_pos.shape[0] - 1}]."
            )

        self.path = motion_path
        self.anchor_body_index = ISAAC_27DOF_BODIES.index(MIMIC_ANCHOR_BODY_NAME)
        self.start_frame = start_frame
        self.frame_index = start_frame
        self.finished = False
        self.anchor_alignment = np.asarray((1.0, 0.0, 0.0, 0.0), dtype=np.float64)
        self._proprio_history = {
            "linear_acceleration": deque(maxlen=MIMIC_PROPRIO_HISTORY_LENGTH),
            "angular_velocity": deque(maxlen=MIMIC_PROPRIO_HISTORY_LENGTH),
            "joint_position": deque(maxlen=MIMIC_PROPRIO_HISTORY_LENGTH),
            "joint_velocity": deque(maxlen=MIMIC_PROPRIO_HISTORY_LENGTH),
        }

    @property
    def frame_count(self) -> int:
        return self.joint_pos.shape[0]

    @property
    def duration(self) -> float:
        return (self.frame_count - self.start_frame) / self.fps

    @property
    def command(self) -> np.ndarray:
        return np.concatenate(
            (self.joint_pos[self.frame_index], self.joint_vel[self.frame_index])
        )

    @property
    def anchor_quaternion(self) -> np.ndarray:
        return quaternion_multiply(
            self.anchor_alignment,
            self.body_quat_w[self.frame_index, self.anchor_body_index],
        )

    def reset(self) -> None:
        self.frame_index = self.start_frame
        self.finished = False
        self.anchor_alignment[:] = (1.0, 0.0, 0.0, 0.0)
        for history in self._proprio_history.values():
            history.clear()

    def update_proprio_history(
        self,
        linear_acceleration: np.ndarray,
        angular_velocity: np.ndarray,
        joint_position: np.ndarray,
        joint_velocity: np.ndarray,
    ) -> np.ndarray:
        """Return Isaac-compatible term-major history, oldest sample first."""
        samples = {
            "linear_acceleration": linear_acceleration,
            "angular_velocity": angular_velocity,
            "joint_position": joint_position,
            "joint_velocity": joint_velocity,
        }
        flattened_history = []
        for name, sample in samples.items():
            sample = np.asarray(sample, dtype=np.float64)
            history = self._proprio_history[name]
            if not history:
                for _ in range(MIMIC_PROPRIO_HISTORY_LENGTH):
                    history.append(sample.copy())
            else:
                history.append(sample.copy())
            flattened_history.append(np.concatenate(tuple(history)))
        return np.concatenate(flattened_history)

    def align_anchor_to(self, robot_anchor_quaternion: np.ndarray) -> None:
        """Rotate the reference clip so its first anchor matches the live robot."""
        reference_anchor = self.body_quat_w[self.start_frame, self.anchor_body_index]
        self.anchor_alignment = quaternion_multiply(
            robot_anchor_quaternion,
            quaternion_conjugate(reference_anchor),
        )

    def advance(self) -> bool:
        """Advance one reference frame and report the first end-of-clip event."""
        if self.frame_index < self.frame_count - 1:
            self.frame_index += 1
            return False
        if self.finished:
            return False
        self.finished = True
        return True


class LeftArmTrajectory:
    """Reproduce the new stand-leftarm safe-entry/exit command."""

    def __init__(
        self,
        traj_path: Path,
        profile: RobotProfile,
        default_joint_pos: np.ndarray,
        enabled: bool,
        period: float = 6.0,
        safe_time: float = 2.0,
        blend_time: float = 2.0,
        ref_vel_scale: float = 0.25,
        start_phase: float = 0.0,
    ):
        if profile.dof != 27:
            raise ValueError("Left-arm trajectory policies require the 27-DOF robot.")
        if not traj_path.is_file():
            raise FileNotFoundError(f"Left-arm trajectory CSV not found: {traj_path}")

        with traj_path.open("r", encoding="utf-8") as file:
            header = file.readline().strip().split(",")
        raw = np.loadtxt(traj_path, delimiter=",", skiprows=1)
        column_indices = [header.index(name) for name in LEFT_ARM_JOINTS]
        samples = raw[:, column_indices]

        sample_count = samples.shape[0]
        coefficients = np.fft.rfft(samples, axis=0)
        self.cos_coeff = (2.0 / sample_count) * coefficients.real
        self.sin_coeff = (-2.0 / sample_count) * coefficients.imag
        self.cos_coeff[0] *= 0.5
        if sample_count % 2 == 0:
            self.cos_coeff[-1] *= 0.5
            self.sin_coeff[-1] = 0.0

        harmonics = np.arange(coefficients.shape[0], dtype=np.float64)
        self.omega = 2.0 * np.pi * harmonics / period
        joint_index = {name: index for index, name in enumerate(profile.joint_names)}
        self.default_q = np.asarray(
            [default_joint_pos[joint_index[name]] for name in LEFT_ARM_JOINTS],
            dtype=np.float64,
        )
        self.safe_q = np.asarray(
            [0.25, 0.15, -1.5707963, -0.6, 0.0, 0.0, 1.5707963],
            dtype=np.float64,
        )
        self.period = period
        self.safe_time = safe_time
        self.blend_time = blend_time
        self.ref_vel_scale = ref_vel_scale
        self.start_phase = start_phase
        self.default_enabled = enabled
        self.enabled = enabled
        self.returning = False
        self.elapsed = 0.0
        self.return_elapsed = 0.0

    def reset(self) -> None:
        self.enabled = self.default_enabled
        self.returning = False
        self.elapsed = 0.0
        self.return_elapsed = 0.0

    def toggle(self) -> None:
        if not self.enabled:
            self.enabled = True
            self.returning = False
            self.elapsed = 0.0
            self.return_elapsed = 0.0
        elif not self.returning:
            self.returning = True
            self.return_elapsed = 0.0

    @staticmethod
    def _smoothstep(elapsed: float, duration: float) -> tuple[float, float]:
        if duration <= 0.0:
            return 1.0, 0.0
        u = float(np.clip(elapsed / duration, 0.0, 1.0))
        alpha = 6.0 * u**5 - 15.0 * u**4 + 10.0 * u**3
        alpha_dot = (
            30.0 * u**4 - 60.0 * u**3 + 30.0 * u**2
        ) / duration
        return alpha, alpha_dot

    def _trajectory_reference(self) -> tuple[np.ndarray, np.ndarray]:
        trajectory_elapsed = max(self.elapsed - self.safe_time, 0.0)
        phase = (self.start_phase + trajectory_elapsed) % self.period
        angles = phase * self.omega
        cosines = np.cos(angles)
        sines = np.sin(angles)
        q_traj = cosines @ self.cos_coeff + sines @ self.sin_coeff
        dq_traj = (
            -(sines * self.omega) @ self.cos_coeff
            + (cosines * self.omega) @ self.sin_coeff
        )
        return q_traj, dq_traj

    def advance(self, dt: float) -> None:
        if not self.enabled:
            return
        self.elapsed += dt
        if self.returning:
            self.return_elapsed += dt
            if self.return_elapsed >= self.blend_time + self.safe_time:
                self.enabled = False
                self.returning = False
                self.elapsed = 0.0
                self.return_elapsed = 0.0

    def observation(self) -> np.ndarray:
        if not self.enabled:
            return np.zeros(LEFT_ARM_COMMAND_DIM, dtype=np.float32)

        q_traj, dq_traj = self._trajectory_reference()
        if self.returning:
            if self.return_elapsed < self.blend_time:
                gamma, gamma_dot = self._smoothstep(
                    self.return_elapsed, self.blend_time
                )
                q_ref = q_traj + gamma * (self.safe_q - q_traj)
                dq_ref = (
                    (1.0 - gamma) * dq_traj
                    + gamma_dot * (self.safe_q - q_traj)
                )
            else:
                eta, eta_dot = self._smoothstep(
                    self.return_elapsed - self.blend_time, self.safe_time
                )
                q_ref = self.safe_q + eta * (self.default_q - self.safe_q)
                dq_ref = eta_dot * (self.default_q - self.safe_q)
        elif self.elapsed < self.safe_time:
            alpha, alpha_dot = self._smoothstep(self.elapsed, self.safe_time)
            safe_delta = self.safe_q - self.default_q
            q_ref = self.default_q + alpha * safe_delta
            dq_ref = alpha_dot * safe_delta
        elif self.elapsed < self.safe_time + self.blend_time:
            beta, beta_dot = self._smoothstep(
                self.elapsed - self.safe_time, self.blend_time
            )
            track_delta = q_traj - self.safe_q
            q_ref = self.safe_q + beta * track_delta
            dq_ref = beta_dot * track_delta + beta * dq_traj
        else:
            q_ref = q_traj
            dq_ref = dq_traj

        return np.concatenate(
            (
                q_ref - self.default_q,
                dq_ref * self.ref_vel_scale,
                np.ones(1),
            )
        ).astype(np.float32)


class TelemetryWaveform:
    """Draw recent Sim2Sim telemetry in a non-blocking OpenCV window."""

    WINDOW_NAME = "Humanoid Ultra - Live Waveforms"
    WIDTH = 1400
    HEIGHT = 1000
    HEADER_HEIGHT = 82
    WINDOW_PRESETS = (1.0, 2.0, 5.0, 10.0, 20.0, 30.0, 60.0, 120.0, 300.0)
    MIN_WINDOW_SECONDS = WINDOW_PRESETS[0]
    MAX_WINDOW_SECONDS = WINDOW_PRESETS[-1]
    BACKGROUND = (242, 242, 242)
    PANEL_BACKGROUND = (255, 255, 255)
    GRID_COLOR = (220, 220, 220)
    TEXT_COLOR = (35, 35, 35)
    BUTTON_COLOR = (232, 232, 232)
    BUTTON_ACTIVE_COLOR = (80, 170, 235)
    BUTTON_BORDER_COLOR = (145, 145, 145)
    COLORS = (
        (200, 70, 40),
        (40, 130, 220),
        (40, 170, 80),
        (180, 70, 180),
        (40, 180, 190),
        (120, 100, 40),
        (230, 150, 60),
        (90, 60, 210),
        (170, 170, 40),
        (70, 190, 150),
        (200, 90, 130),
        (120, 120, 210),
        (80, 80, 80),
        (150, 70, 30),
        (30, 150, 150),
        (150, 40, 100),
    )

    def __init__(
        self,
        simulator,
        window_seconds: float,
        refresh_hz: float,
        joint_names: tuple[str, ...],
        torque_joint_names: tuple[str, ...] = (),
        save_directory: Path | None = None,
        show_window: bool = True,
    ):
        import cv2

        if refresh_hz <= 0.0:
            raise ValueError("Waveform refresh rate must be positive.")
        if not self.MIN_WINDOW_SECONDS <= window_seconds <= self.MAX_WINDOW_SECONDS:
            raise ValueError(
                "Waveform window must be between "
                f"{self.MIN_WINDOW_SECONDS:g} and "
                f"{self.MAX_WINDOW_SECONDS:g} seconds."
            )
        preferred_joints = (
            "left_knee_joint",
            "right_knee_joint",
            "left_shoulder_roll_joint",
            "left_wrist_yaw_joint",
        )
        if not joint_names:
            joint_names = tuple(
                name for name in preferred_joints if name in simulator.profile.joint_names
            )
        missing = [name for name in joint_names if name not in simulator.profile.joint_names]
        if missing:
            raise ValueError(
                "Unknown --plot-joints {}. Available joints: {}".format(
                    missing, ", ".join(simulator.profile.joint_names)
                )
            )
        if not 1 <= len(joint_names) <= len(self.COLORS):
            raise ValueError(
                f"--plot-joints must select between 1 and {len(self.COLORS)} joints."
            )
        preferred_torque_joints = (
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_hip_pitch_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_hip_pitch_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
        )
        if not torque_joint_names:
            torque_joint_names = tuple(
                name
                for name in preferred_torque_joints
                if name in simulator.profile.joint_names
            )
        missing_torque = [
            name
            for name in torque_joint_names
            if name not in simulator.profile.joint_names
        ]
        if missing_torque:
            raise ValueError(
                "Unknown --plot-torque-joints {}. Available joints: {}".format(
                    missing_torque, ", ".join(simulator.profile.joint_names)
                )
            )
        if not 1 <= len(torque_joint_names) <= len(self.COLORS):
            raise ValueError(
                "--plot-torque-joints must select between 1 and "
                f"{len(self.COLORS)} joints."
            )

        self.cv2 = cv2
        self.window_seconds = float(window_seconds)
        self.refresh_period = 1.0 / float(refresh_hz)
        self.save_directory = (
            SCRIPT_DIR / "waveform_data"
            if save_directory is None
            else Path(save_directory)
        )
        self.joint_names = tuple(joint_names)
        self.torque_joint_names = tuple(torque_joint_names)
        self.joint_indices = {
            name: simulator.profile.joint_names.index(name) for name in self.joint_names
        }
        self.torque_joint_indices = {
            name: simulator.profile.joint_names.index(name)
            for name in self.torque_joint_names
        }
        max_points = max(
            2,
            int(
                np.ceil(
                    self.MAX_WINDOW_SECONDS
                    / (simulator.SIM_DT * simulator.CONTROL_DECIMATION)
                )
            )
            + 1,
        )
        self.times = deque(maxlen=max_points)
        self.states = deque(maxlen=max_points)
        self.policies = deque(maxlen=max_points)
        self.base_z = deque(maxlen=max_points)
        self.rpy = [deque(maxlen=max_points) for _ in range(3)]
        self.body_velocity = [deque(maxlen=max_points) for _ in range(3)]
        self.gyro = [deque(maxlen=max_points) for _ in range(3)]
        self.command = [deque(maxlen=max_points) for _ in range(3)]
        self.error_rms = deque(maxlen=max_points)
        self.error_max = deque(maxlen=max_points)
        self.joint_actual = {
            name: deque(maxlen=max_points) for name in self.joint_names
        }
        self.joint_target = {
            name: deque(maxlen=max_points) for name in self.joint_names
        }
        self.joint_torque = {
            name: deque(maxlen=max_points) for name in self.torque_joint_names
        }
        self.last_draw_time = -np.inf
        self.last_frame = np.full(
            (self.HEIGHT, self.WIDTH, 3), self.BACKGROUND, dtype=np.uint8
        )
        self.paused = False
        self.pending_actions = deque()
        self.redraw_requested = True
        self.status_message = "50 Hz sampling"
        self.button_rects: dict[str, tuple[int, int, int, int]] = {}
        self.show_window = bool(show_window)
        self.render_when_hidden = not self.show_window
        if self.show_window:
            try:
                self.cv2.namedWindow(self.WINDOW_NAME, self.cv2.WINDOW_NORMAL)
                self.cv2.resizeWindow(self.WINDOW_NAME, self.WIDTH, self.HEIGHT)
                self.cv2.imshow(self.WINDOW_NAME, self.last_frame)
                self.cv2.setMouseCallback(
                    self.WINDOW_NAME, self._mouse_callback
                )
                self.cv2.waitKey(1)
            except self.cv2.error as exc:
                raise RuntimeError(
                    "Cannot open the waveform window. Run from a graphical desktop "
                    "or omit --plot."
                ) from exc

    @staticmethod
    def _short_joint_name(name: str) -> str:
        return (
            name.replace("left_", "L_")
            .replace("right_", "R_")
            .replace("_joint", "")
            .replace("_", " ")
        )

    def _clear_data(self) -> None:
        self.times.clear()
        self.states.clear()
        self.policies.clear()
        self.base_z.clear()
        self.error_rms.clear()
        self.error_max.clear()
        for group in (self.rpy, self.body_velocity, self.gyro, self.command):
            for values in group:
                values.clear()
        for group in (
            self.joint_actual,
            self.joint_target,
            self.joint_torque,
        ):
            for values in group.values():
                values.clear()

    def _mouse_callback(self, event, x, y, _flags, _param) -> None:
        if event != self.cv2.EVENT_LBUTTONUP:
            return
        for action, (x0, y0, x1, y1) in self.button_rects.items():
            if x0 <= x <= x1 and y0 <= y <= y1:
                self.pending_actions.append(action)
                return

    def _change_window(self, direction: int) -> None:
        if direction < 0:
            candidates = [
                value
                for value in self.WINDOW_PRESETS
                if value < self.window_seconds - 1.0e-6
            ]
            new_value = candidates[-1] if candidates else self.WINDOW_PRESETS[0]
        else:
            candidates = [
                value
                for value in self.WINDOW_PRESETS
                if value > self.window_seconds + 1.0e-6
            ]
            new_value = candidates[0] if candidates else self.WINDOW_PRESETS[-1]
        self.window_seconds = float(new_value)
        self.status_message = f"Display window: {self.window_seconds:g} s"
        self.redraw_requested = True

    def _save_csv(self) -> None:
        if not self.times:
            self.status_message = "No waveform data to save"
            self.redraw_requested = True
            return

        import csv
        from datetime import datetime

        times = np.asarray(self.times, dtype=np.float64)
        first_index = int(
            np.searchsorted(times, times[-1] - self.window_seconds, side="left")
        )
        states = list(self.states)
        policies = list(self.policies)
        command = [np.asarray(values) for values in self.command]
        base_z = np.asarray(self.base_z)
        rpy = [np.asarray(values) for values in self.rpy]
        body_velocity = [np.asarray(values) for values in self.body_velocity]
        gyro = [np.asarray(values) for values in self.gyro]
        error_rms = np.asarray(self.error_rms)
        error_max = np.asarray(self.error_max)
        joint_actual = {
            name: np.asarray(values) for name, values in self.joint_actual.items()
        }
        joint_target = {
            name: np.asarray(values) for name, values in self.joint_target.items()
        }
        joint_torque = {
            name: np.asarray(values) for name, values in self.joint_torque.items()
        }
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        path = self.save_directory / f"humanoid_ultra_waveform_{timestamp}.csv"
        header = [
            "time_s",
            "control_state",
            "policy",
            "command_c0",
            "command_c1",
            "command_c2",
            "base_height_m",
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
            "body_vx_m_s",
            "body_vy_m_s",
            "body_vz_m_s",
            "body_wx_rad_s",
            "body_wy_rad_s",
            "body_wz_rad_s",
            "pd_error_rms_rad",
            "pd_error_max_abs_rad",
        ]
        for name in self.joint_names:
            header.extend(
                (
                    f"{name}_position_rad",
                    f"{name}_pd_target_rad",
                )
            )
        for name in self.torque_joint_names:
            header.append(f"{name}_actuator_torque_nm")

        try:
            self.save_directory.mkdir(parents=True, exist_ok=True)
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(header)
                for index in range(first_index, len(times)):
                    row = [
                        times[index],
                        states[index],
                        policies[index],
                        command[0][index],
                        command[1][index],
                        command[2][index],
                        base_z[index],
                        rpy[0][index],
                        rpy[1][index],
                        rpy[2][index],
                        body_velocity[0][index],
                        body_velocity[1][index],
                        body_velocity[2][index],
                        gyro[0][index],
                        gyro[1][index],
                        gyro[2][index],
                        error_rms[index],
                        error_max[index],
                    ]
                    for name in self.joint_names:
                        row.extend(
                            (
                                joint_actual[name][index],
                                joint_target[name][index],
                            )
                        )
                    for name in self.torque_joint_names:
                        row.append(joint_torque[name][index])
                    writer.writerow(row)
        except OSError as exc:
            self.status_message = f"Save failed: {exc}"
            print(f"Waveform CSV save failed: {exc}")
        else:
            sample_count = len(times) - first_index
            self.status_message = f"Saved {sample_count} samples: {path.name}"
            print(f"Waveform CSV saved: {path} ({sample_count} samples)")
        self.redraw_requested = True

    def _process_pending_actions(self) -> None:
        while self.pending_actions:
            action = self.pending_actions.popleft()
            if action == "window_down":
                self._change_window(-1)
            elif action == "window_up":
                self._change_window(1)
            elif action == "pause":
                self.paused = not self.paused
                self.status_message = (
                    "Waveform sampling paused; simulation still running"
                    if self.paused
                    else "Waveform sampling resumed"
                )
                self.redraw_requested = True
            elif action == "save":
                self._save_csv()

    def sample(self, simulator) -> None:
        self._process_pending_actions()
        sim_time = float(simulator.data.time)
        if self.times and sim_time < self.times[-1]:
            self._clear_data()
            self.last_draw_time = -np.inf
            self.status_message = "Simulation reset; waveform buffer cleared"
            self.redraw_requested = True

        if not self.paused:
            joint_pos = simulator.data.qpos[simulator.qpos_indices].copy()
            joint_target = simulator.target_joint_pos.copy()
            joint_error = joint_target - joint_pos
            body_rotation = quaternion_to_rotation_matrix(simulator.data.qpos[3:7])
            roll = np.arctan2(body_rotation[2, 1], body_rotation[2, 2])
            pitch = np.arctan2(
                -body_rotation[2, 0],
                np.hypot(body_rotation[2, 1], body_rotation[2, 2]),
            )
            yaw = np.arctan2(body_rotation[1, 0], body_rotation[0, 0])
            rpy_deg = np.rad2deg((roll, pitch, yaw))
            body_velocity = simulator.data.sensor("BodyVel").data.copy()
            gyro = simulator.data.sensor("BodyGyro").data.copy()
            joint_torque = simulator.data.qfrc_actuator[
                simulator.qvel_indices
            ].copy()

            self.times.append(sim_time)
            self.states.append(simulator.control_state)
            self.policies.append(simulator.active_name)
            self.base_z.append(float(simulator.data.qpos[2]))
            for index in range(3):
                self.rpy[index].append(float(rpy_deg[index]))
                self.body_velocity[index].append(float(body_velocity[index]))
                self.gyro[index].append(float(gyro[index]))
                self.command[index].append(float(simulator.command[index]))
            self.error_rms.append(float(np.sqrt(np.mean(np.square(joint_error)))))
            self.error_max.append(float(np.max(np.abs(joint_error))))
            for name, index in self.joint_indices.items():
                self.joint_actual[name].append(float(joint_pos[index]))
                self.joint_target[name].append(float(joint_target[index]))
            for name, index in self.torque_joint_indices.items():
                self.joint_torque[name].append(float(joint_torque[index]))

        draw_due = sim_time - self.last_draw_time >= self.refresh_period
        if draw_due and (self.show_window or self.render_when_hidden):
            if not self.paused or self.redraw_requested:
                self.last_frame = self.render_frame(simulator)
                self.redraw_requested = False
            self.last_draw_time = sim_time
            if self.show_window:
                self._show()

    def _show(self) -> None:
        try:
            visible = self.cv2.getWindowProperty(
                self.WINDOW_NAME, self.cv2.WND_PROP_VISIBLE
            )
        except self.cv2.error:
            visible = 0.0
        if visible < 1.0:
            self.show_window = False
            return
        self.cv2.imshow(self.WINDOW_NAME, self.last_frame)
        key = self.cv2.waitKey(1) & 0xFF
        if key == 27:  # Escape closes only the waveform window.
            self.close()
        elif key in (ord("-"), ord("_"), ord("[")):
            self.pending_actions.append("window_down")
        elif key in (ord("+"), ord("="), ord("]")):
            self.pending_actions.append("window_up")
        elif key == ord(" "):
            self.pending_actions.append("pause")
        elif key in (ord("s"), ord("S")):
            self.pending_actions.append("save")

    def _draw_control_button(
        self,
        image,
        action: str | None,
        label: str,
        rect: tuple[int, int, int, int],
        active: bool = False,
    ) -> None:
        x0, y0, width, height = rect
        color = self.BUTTON_ACTIVE_COLOR if active else self.BUTTON_COLOR
        self.cv2.rectangle(
            image, (x0, y0), (x0 + width, y0 + height), color, thickness=-1
        )
        self.cv2.rectangle(
            image,
            (x0, y0),
            (x0 + width, y0 + height),
            self.BUTTON_BORDER_COLOR,
            thickness=1,
        )
        text_size, _ = self.cv2.getTextSize(
            label, self.cv2.FONT_HERSHEY_SIMPLEX, 0.43, 1
        )
        text_x = x0 + max(5, (width - text_size[0]) // 2)
        text_y = y0 + (height + text_size[1]) // 2
        self.cv2.putText(
            image,
            label,
            (text_x, text_y),
            self.cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            self.TEXT_COLOR,
            1,
            self.cv2.LINE_AA,
        )
        if action is not None:
            self.button_rects[action] = (x0, y0, x0 + width, y0 + height)

    def _draw_header(self, image, simulator) -> None:
        self.button_rects.clear()
        state_color = (40, 80, 210) if self.paused else (40, 145, 60)
        state_label = "PAUSED" if self.paused else "RECORDING"
        self.cv2.circle(image, (27, 25), 7, state_color, thickness=-1)
        self.cv2.putText(
            image,
            (
                f"{state_label}   state={simulator.control_state}   "
                f"policy={simulator.active_name}"
            ),
            (42, 31),
            self.cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            self.TEXT_COLOR,
            1,
            self.cv2.LINE_AA,
        )
        self.cv2.putText(
            image,
            "Joint position: solid=actual, dash=PD target",
            (930, 30),
            self.cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (85, 85, 85),
            1,
            self.cv2.LINE_AA,
        )

        y = 43
        self._draw_control_button(
            image, "window_down", "Window -  [-]", (20, y, 118, 30)
        )
        self._draw_control_button(
            image, None, f"{self.window_seconds:g} s", (146, y, 76, 30)
        )
        self._draw_control_button(
            image, "window_up", "Window +  [+]", (230, y, 118, 30)
        )
        self._draw_control_button(
            image,
            "pause",
            "Resume  [Space]" if self.paused else "Pause  [Space]",
            (362, y, 154, 30),
            active=self.paused,
        )
        self._draw_control_button(
            image, "save", "Save CSV  [S]", (524, y, 134, 30)
        )
        message = self.status_message
        if len(message) > 90:
            message = message[:87] + "..."
        self.cv2.putText(
            image,
            message,
            (675, 63),
            self.cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (75, 75, 75),
            1,
            self.cv2.LINE_AA,
        )

    def _draw_panel(self, image, rect, title, series) -> None:
        x0, y0, width, height = rect
        cv2 = self.cv2
        cv2.rectangle(
            image,
            (x0, y0),
            (x0 + width, y0 + height),
            self.PANEL_BACKGROUND,
            thickness=-1,
        )
        cv2.rectangle(
            image,
            (x0, y0),
            (x0 + width, y0 + height),
            (180, 180, 180),
            thickness=1,
        )
        cv2.putText(
            image,
            title,
            (x0 + 10, y0 + 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            self.TEXT_COLOR,
            1,
            cv2.LINE_AA,
        )

        labeled = [(label, color) for label, _, color, _ in series if label]
        legend_x = x0 + 10
        legend_y = y0 + 38
        for label, color in labeled:
            legend_width = max(80, 32 + 7 * len(label))
            if legend_x + legend_width > x0 + width - 10:
                legend_x = x0 + 10
                legend_y += 18
            cv2.line(
                image,
                (legend_x, legend_y),
                (legend_x + 18, legend_y),
                color,
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                image,
                label,
                (legend_x + 23, legend_y + 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                self.TEXT_COLOR,
                1,
                cv2.LINE_AA,
            )
            legend_x += legend_width

        plot_left = x0 + 52
        plot_right = x0 + width - 12
        plot_top = max(y0 + 52, legend_y + 14)
        plot_bottom = y0 + height - 30
        if len(self.times) < 2:
            cv2.putText(
                image,
                "waiting for samples...",
                (plot_left + 20, plot_top + 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (120, 120, 120),
                1,
                cv2.LINE_AA,
            )
            return

        times = np.asarray(self.times, dtype=np.float64)
        time_max = times[-1]
        time_min = max(times[0], time_max - self.window_seconds)
        mask = times >= time_min
        times = times[mask]
        segment_ids = np.cumsum(
            np.concatenate(
                (
                    np.zeros(1, dtype=np.int32),
                    (np.diff(times) > 0.1).astype(np.int32),
                )
            )
        )
        values_by_series = []
        finite_values = []
        for _, values, _, _ in series:
            array = np.asarray(values, dtype=np.float64)[mask]
            values_by_series.append(array)
            finite_values.extend(array[np.isfinite(array)].tolist())
        if not finite_values:
            return
        value_min = float(np.min(finite_values))
        value_max = float(np.max(finite_values))
        if np.isclose(value_min, value_max):
            padding = max(0.05, abs(value_min) * 0.1)
        else:
            padding = 0.1 * (value_max - value_min)
        value_min -= padding
        value_max += padding

        max_draw_points = max(200, 2 * (plot_right - plot_left))
        if len(times) > max_draw_points:
            draw_indices = np.linspace(
                0, len(times) - 1, max_draw_points, dtype=np.int32
            )
            draw_indices = np.unique(draw_indices)
            times = times[draw_indices]
            segment_ids = segment_ids[draw_indices]
            values_by_series = [
                values[draw_indices] for values in values_by_series
            ]

        for fraction in np.linspace(0.0, 1.0, 5):
            x = int(plot_left + fraction * (plot_right - plot_left))
            y = int(plot_top + fraction * (plot_bottom - plot_top))
            cv2.line(
                image, (x, plot_top), (x, plot_bottom), self.GRID_COLOR, 1
            )
            cv2.line(
                image, (plot_left, y), (plot_right, y), self.GRID_COLOR, 1
            )

        cv2.putText(
            image,
            f"{value_max:.3g}",
            (x0 + 3, plot_top + 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            self.TEXT_COLOR,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            f"{value_min:.3g}",
            (x0 + 3, plot_bottom + 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            self.TEXT_COLOR,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            f"{time_min:.1f}s",
            (plot_left, y0 + height - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            self.TEXT_COLOR,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            f"{time_max:.1f}s",
            (plot_right - 42, y0 + height - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            self.TEXT_COLOR,
            1,
            cv2.LINE_AA,
        )

        time_span = max(time_max - time_min, 1.0e-6)
        value_span = max(value_max - value_min, 1.0e-6)
        x_pixels = plot_left + (times - time_min) / time_span * (
            plot_right - plot_left
        )
        for (_, _, color, style), values in zip(series, values_by_series):
            y_pixels = plot_bottom - (values - value_min) / value_span * (
                plot_bottom - plot_top
            )
            for index in range(1, len(times)):
                if not (
                    np.isfinite(y_pixels[index - 1])
                    and np.isfinite(y_pixels[index])
                ):
                    continue
                if segment_ids[index] != segment_ids[index - 1]:
                    continue
                if style == "dash" and (index // 4) % 2:
                    continue
                if style == "dot" and index % 4:
                    continue
                cv2.line(
                    image,
                    (int(x_pixels[index - 1]), int(y_pixels[index - 1])),
                    (int(x_pixels[index]), int(y_pixels[index])),
                    color,
                    2 if style == "solid" else 1,
                    cv2.LINE_AA,
                )

    def render_frame(self, simulator) -> np.ndarray:
        image = np.full(
            (self.HEIGHT, self.WIDTH, 3), self.BACKGROUND, dtype=np.uint8
        )
        self._draw_header(image, simulator)
        margin = 18
        header = self.HEADER_HEIGHT
        gap = 14
        panel_width = (self.WIDTH - 2 * margin - gap) // 2
        torque_panel_height = 245
        panel_height = (
            self.HEIGHT
            - header
            - margin
            - torque_panel_height
            - 3 * gap
        ) // 3
        panels = []
        for row in range(3):
            for column in range(2):
                panels.append(
                    (
                        margin + column * (panel_width + gap),
                        header + row * (panel_height + gap),
                        panel_width,
                        panel_height,
                    )
                )
        torque_y = header + 3 * panel_height + 3 * gap
        torque_panel = (
            margin,
            torque_y,
            self.WIDTH - 2 * margin,
            self.HEIGHT - margin - torque_y,
        )

        self._draw_panel(
            image,
            panels[0],
            "Policy command [c0, c1, c2]",
            tuple(
                (f"c{index}", values, self.COLORS[index], "solid")
                for index, values in enumerate(self.command)
            ),
        )
        self._draw_panel(
            image,
            panels[1],
            "Base height [m]",
            (("base z", self.base_z, self.COLORS[0], "solid"),),
        )
        self._draw_panel(
            image,
            panels[2],
            "Base orientation [deg]",
            tuple(
                (name, values, color, "solid")
                for name, values, color in zip(
                    ("roll", "pitch", "yaw"), self.rpy, self.COLORS
                )
            ),
        )
        self._draw_panel(
            image,
            panels[3],
            "Body linear velocity [m/s]",
            tuple(
                (axis, values, color, "solid")
                for axis, values, color in zip(
                    ("vx", "vy", "vz"), self.body_velocity, self.COLORS
                )
            ),
        )
        self._draw_panel(
            image,
            panels[4],
            "Body angular velocity [rad/s]",
            tuple(
                (axis, values, color, "solid")
                for axis, values, color in zip(
                    ("wx", "wy", "wz"), self.gyro, self.COLORS
                )
            ),
        )
        joint_series = []
        for joint_number, name in enumerate(self.joint_names):
            color = self.COLORS[joint_number]
            joint_series.extend(
                (
                    (
                        self._short_joint_name(name),
                        self.joint_actual[name],
                        color,
                        "solid",
                    ),
                    ("", self.joint_target[name], color, "dash"),
                )
            )
        self._draw_panel(
            image,
            panels[5],
            "Selected joint position [rad]",
            tuple(joint_series),
        )
        torque_series = tuple(
            (
                self._short_joint_name(name),
                self.joint_torque[name],
                self.COLORS[joint_number],
                "solid",
            )
            for joint_number, name in enumerate(self.torque_joint_names)
        )
        self._draw_panel(
            image,
            torque_panel,
            "Leg joint actuator torque [N m]",
            torque_series,
        )
        return image

    def close(self) -> None:
        if not self.show_window:
            return
        self.show_window = False
        try:
            self.cv2.destroyWindow(self.WINDOW_NAME)
            self.cv2.waitKey(1)
        except self.cv2.error:
            pass


class ElasticBand:
    """Apply two spring-damper suspension forces at the shoulder sockets."""

    GRAVITY = 9.81
    SHOULDER_BODY_NAMES = (
        "left_shoulder_pitch_link",
        "right_shoulder_pitch_link",
    )

    def __init__(
        self,
        model: mujoco.MjModel,
        suspension_height: float,
        anchor_height: float,
        stiffness: float,
        damping: float,
        support_ratio: float,
        enabled: bool,
    ):
        if anchor_height <= suspension_height:
            raise ValueError("--band-anchor-height must be above the suspended robot height.")
        if stiffness <= 0.0 or damping < 0.0:
            raise ValueError("Elastic-band stiffness must be positive and damping non-negative.")
        if not 0.0 <= support_ratio <= 1.0:
            raise ValueError("--band-support-ratio must be between 0.0 and 1.0.")

        self.model = model
        self.trunk_body_id = model.body("trunk_link").id
        self.shoulder_body_ids = tuple(
            model.body(name).id for name in self.SHOULDER_BODY_NAMES
        )
        self.anchor_height = anchor_height
        self.points = np.zeros((2, 3), dtype=np.float64)
        self.stiffness = stiffness
        self.damping = damping
        self.support_ratio = support_ratio
        self.enabled = enabled
        self.suspension_height = suspension_height
        self.robot_weight = float(np.sum(model.body_mass)) * self.GRAVITY
        self.length = 0.0
        self.max_force_per_band = 1.25 * self.robot_weight
        self._body_velocity = np.zeros(6, dtype=np.float64)
        self._zero_torque = np.zeros(3, dtype=np.float64)

    def reset_anchors(self, data: mujoco.MjData) -> None:
        shoulder_positions = np.asarray(
            [data.xpos[body_id].copy() for body_id in self.shoulder_body_ids]
        )
        self.points[:, :2] = shoulder_positions[:, :2]
        self.points[:, 2] = self.anchor_height
        anchor_distance = float(self.anchor_height - np.mean(shoulder_positions[:, 2]))
        force_per_band = 0.5 * self.robot_weight * self.support_ratio
        self.length = max(0.0, anchor_distance - force_per_band / self.stiffness)

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = enabled

    def toggle(self) -> None:
        self.enabled = not self.enabled

    def adjust_length(self, delta: float) -> None:
        self.length = max(0.0, self.length + delta)

    def apply(self, data: mujoco.MjData) -> None:
        data.qfrc_applied[:] = 0.0
        if not self.enabled:
            return

        for anchor, shoulder_body_id in zip(self.points, self.shoulder_body_ids):
            shoulder_position = data.xpos[shoulder_body_id]
            displacement = anchor - shoulder_position
            distance = float(np.linalg.norm(displacement))
            if distance < 1.0e-9:
                continue

            direction = displacement / distance
            mujoco.mj_objectVelocity(
                self.model,
                data,
                mujoco.mjtObj.mjOBJ_BODY,
                shoulder_body_id,
                self._body_velocity,
                0,
            )
            velocity_along_band = float(np.dot(self._body_velocity[3:], direction))
            extension = max(0.0, distance - self.length)
            force_magnitude = self.stiffness * extension - self.damping * velocity_along_band
            force_magnitude = float(np.clip(force_magnitude, 0.0, self.max_force_per_band))
            mujoco.mj_applyFT(
                self.model,
                data,
                force_magnitude * direction,
                self._zero_torque,
                shoulder_position,
                self.trunk_body_id,
                data.qfrc_applied,
            )


class HumanoidUltraSim2Sim:
    SIM_DT = 0.005
    CONTROL_DECIMATION = 4
    ACTION_SCALE = 0.25
    HISTORY_LENGTH = 10
    POLICY_BLEND_DURATION = 0.5
    # Uniform joint damping used by the LT+B kd-only stop.
    DAMPING_STOP_KD = 5.0

    def __init__(
        self,
        dof: int,
        policy_specs: list[PolicySpec],
        command: np.ndarray,
        mimic_action_specs: list[MimicActionSpec],
        left_arm_traj_path: Path | None,
        left_arm_enabled: bool,
        left_arm_start_phase: float,
        elastic_band_enabled: bool,
        band_lift: float,
        band_anchor_height: float,
        band_stiffness: float,
        band_damping: float,
        band_support_ratio: float,
        scene_path: Path | None = None,
    ):
        self.profile = make_profile(dof)
        repository_root = Path(__file__).resolve().parents[2]
        # The default scene was generated from the nominal CAD URDF.  Training
        # uses the system-identified one, and the payload tasks use an
        # identified model with the left-hand load; both totals land within
        # 0.04 kg of nominal but the per-link masses differ by 3.1 kg in
        # aggregate, so evaluating a payload policy on the default scene is a
        # different robot.  scene_27dof_identified.xml and
        # scene_27dof_identified_leftarm2p5kg.xml carry those inertials.
        model_path = Path(scene_path) if scene_path is not None else (
            repository_root
            / "unitree_robots"
            / "humanoid_ultra"
            / f"scene_{self.profile.dof}dof.xml"
        )
        if not model_path.is_file():
            raise FileNotFoundError(f"MuJoCo scene not found: {model_path}")
        if not policy_specs:
            raise ValueError("At least one policy must be provided.")
        for spec in policy_specs:
            if not spec.path.is_file():
                raise FileNotFoundError(f"Exported TorchScript policy not found: {spec.path}")
        action_names = [spec.name for spec in mimic_action_specs]
        action_triggers = [spec.trigger for spec in mimic_action_specs if spec.trigger]
        action_keyboard_keys = [
            spec.keyboard_key.lower()
            for spec in mimic_action_specs
            if spec.keyboard_key
        ]
        if len(action_names) != len(set(action_names)):
            raise ValueError("Mimic action names must be unique.")
        if len(action_triggers) != len(set(action_triggers)):
            raise ValueError("Mimic action gamepad triggers must be unique.")
        if any(
            len(spec.keyboard_key) != 1
            for spec in mimic_action_specs
            if spec.keyboard_key
        ):
            raise ValueError("Mimic action keyboard keys must be single characters.")
        if len(action_keyboard_keys) != len(set(action_keyboard_keys)):
            raise ValueError("Mimic action keyboard keys must be unique.")
        for spec in mimic_action_specs:
            if not spec.motion_path.is_file():
                raise FileNotFoundError(
                    f"Mimic motion NPZ not found ({spec.name}): {spec.motion_path}"
                )
            if spec.start_frame < 0:
                raise ValueError(f"Mimic start frame must be non-negative ({spec.name}).")
            if spec.target_speed <= 0.0:
                raise ValueError(f"Mimic target speed must be positive ({spec.name}).")

        self.model_path = model_path
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = self.SIM_DT
        self.elastic_band_default_enabled = elastic_band_enabled
        self.elastic_band = ElasticBand(
            model=self.model,
            suspension_height=self.profile.root_height + band_lift,
            anchor_height=band_anchor_height,
            stiffness=band_stiffness,
            damping=band_damping,
            support_ratio=band_support_ratio,
            enabled=elastic_band_enabled,
        )

        actuator_names = tuple(self.model.actuator(index).name for index in range(self.model.nu))
        if set(actuator_names) != set(self.profile.joint_names):
            raise RuntimeError(
                "MuJoCo actuators do not match the Isaac Lab policy joints.\n"
                f"Expected: {sorted(self.profile.joint_names)}\nActual:   {sorted(actuator_names)}"
            )

        self.actuator_indices = np.asarray(
            [self.model.actuator(name).id for name in self.profile.joint_names], dtype=np.int32
        )
        self.qpos_indices = np.asarray(
            [self.model.joint(name).qposadr[0] for name in self.profile.joint_names], dtype=np.int32
        )
        self.qvel_indices = np.asarray(
            [self.model.joint(name).dofadr[0] for name in self.profile.joint_names], dtype=np.int32
        )
        for name, limits in zip(self.profile.joint_names, self.profile.position_limits):
            self.model.jnt_range[self.model.joint(name).id] = limits
        self.policy_entries = []
        for spec in policy_specs:
            policy = torch.jit.load(str(spec.path), map_location="cpu")
            policy.eval()
            input_dim = policy_input_dim(policy)
            uses_mimic = input_dim in self.profile.mimic_observation_dims
            uses_mimic_history = input_dim == self.profile.mimic_history_observation_dim
            if uses_mimic:
                if spec.mode != "mimic":
                    raise RuntimeError(
                        f"Policy {spec.name} has the {input_dim}-D Mimic input, so run it with "
                        "--mode mimic and --motion-file /path/to/motion.npz."
                    )
                uses_left_arm = False
            else:
                if spec.mode == "mimic":
                    raise RuntimeError(
                        f"Policy {spec.name} has input size {input_dim}, but a 27-DOF Mimic "
                        f"policy must have one of {self.profile.mimic_observation_dims} inputs."
                    )
                if input_dim % self.HISTORY_LENGTH != 0:
                    raise RuntimeError(
                        f"Policy input size {input_dim} is not divisible by history length "
                        f"{self.HISTORY_LENGTH}."
                    )
                policy_frame_dim = input_dim // self.HISTORY_LENGTH
                if policy_frame_dim == self.profile.observation_dim:
                    uses_left_arm = False
                elif policy_frame_dim == self.profile.observation_dim + LEFT_ARM_COMMAND_DIM:
                    uses_left_arm = True
                else:
                    raise RuntimeError(
                        f"Unsupported policy observation size ({spec.name}): "
                        f"{policy_frame_dim} per frame. Expected "
                        f"{self.profile.observation_dim} for standard stand/locomotion, "
                        f"{self.profile.observation_dim + LEFT_ARM_COMMAND_DIM} for stand-leftarm, "
                        f"or one of {self.profile.mimic_observation_dims} total for Mimic."
                    )
            with torch.inference_mode():
                test_output = policy(torch.zeros(1, input_dim, dtype=torch.float32))
            if not isinstance(test_output, torch.Tensor) or tuple(test_output.shape) != (
                1,
                self.profile.dof,
            ):
                raise RuntimeError(
                    f"Policy shape mismatch ({spec.name}): expected [1, {self.profile.dof}], "
                    f"got {getattr(test_output, 'shape', type(test_output))}"
                )
            default_joint_pos = self.profile.default_joint_pos.copy()
            if uses_left_arm or uses_mimic_history or spec.use_current_asset_defaults:
                joint_index = {
                    name: index for index, name in enumerate(self.profile.joint_names)
                }
                # The observation size no longer identifies the zero point:
                # newly trained Mimic policies are also 144-D. Keep it explicit
                # so legacy 144-D policies remain reproducible.
                default_joint_pos[joint_index["left_shoulder_roll_joint"]] = 0.10
                default_joint_pos[joint_index["right_shoulder_roll_joint"]] = -0.10
            self.policy_entries.append(
                {
                    "name": spec.name,
                    "mode": spec.mode,
                    "policy": policy,
                    "uses_left_arm": uses_left_arm,
                    "uses_mimic": uses_mimic,
                    "uses_mimic_history": uses_mimic_history,
                    "default_joint_pos": default_joint_pos,
                }
            )
        self.active_policy_index = 0

        self.mimic_actions = {}
        for action_spec in mimic_action_specs:
            policy_index = self._policy_index_by_name(action_spec.name)
            if policy_index is None:
                raise ValueError(
                    f"Mimic action {action_spec.name!r} has no matching policy."
                )
            if not self.policy_entries[policy_index]["uses_mimic"]:
                raise ValueError(
                    f"Mimic action {action_spec.name!r} does not use Mimic observations."
                )
            self.mimic_actions[action_spec.name] = {
                "spec": action_spec,
                "policy_index": policy_index,
                "motion": MimicMotion(
                    action_spec.motion_path.resolve(),
                    self.profile,
                    action_spec.start_frame,
                ),
            }

        if any(entry["uses_left_arm"] for entry in self.policy_entries):
            left_arm_entry = next(
                entry for entry in self.policy_entries if entry["uses_left_arm"]
            )
            trajectory_path = left_arm_traj_path or Path(__file__).with_name(
                "left_wrist_pitch_traj.csv"
            )
            self.left_arm_trajectory = LeftArmTrajectory(
                trajectory_path.resolve(),
                self.profile,
                left_arm_entry["default_joint_pos"],
                left_arm_enabled,
                start_phase=left_arm_start_phase,
            )
        else:
            self.left_arm_trajectory = None
        self.command = command.astype(np.float64)
        self.previous_action = np.zeros(self.profile.dof, dtype=np.float64)
        self.target_joint_pos = self.policy_entries[0]["default_joint_pos"].copy()
        self.observation_history: deque[np.ndarray] = deque(maxlen=self.HISTORY_LENGTH)
        self.policy_transition_start: np.ndarray | None = None
        self.policy_transition_elapsed = 0.0
        self.damping_stopped = False

        self.reset()

    @property
    def active_entry(self) -> dict:
        return self.policy_entries[self.active_policy_index]

    @property
    def active_name(self) -> str:
        return self.active_entry["name"]

    @property
    def active_mode(self) -> str:
        return self.active_entry["mode"]

    @property
    def active_default_joint_pos(self) -> np.ndarray:
        return self.active_entry["default_joint_pos"]

    @property
    def control_state(self) -> str:
        """Human-readable policy state."""
        if self.active_entry["uses_mimic"]:
            return f"{self.active_name}_play"
        return self.active_name

    @property
    def mimic_in_progress(self) -> bool:
        return self.active_entry["uses_mimic"]

    @property
    def accepts_motion_commands(self) -> bool:
        return not self.mimic_in_progress and self.active_mode in ("stand", "locomotion")

    @property
    def has_mimic_policy(self) -> bool:
        return bool(self.mimic_actions)

    @property
    def active_mimic_action(self) -> dict:
        if not self.active_entry["uses_mimic"]:
            raise RuntimeError("The active policy is not a Mimic action.")
        return self.mimic_actions[self.active_name]

    @property
    def active_mimic_motion(self) -> MimicMotion:
        return self.active_mimic_action["motion"]

    @property
    def mimic_trigger_map(self) -> dict[str, str]:
        return {
            action["spec"].trigger: name
            for name, action in self.mimic_actions.items()
            if action["spec"].trigger
        }

    @property
    def mimic_keyboard_map(self) -> dict[str, str]:
        return {
            action["spec"].keyboard_key.lower(): name
            for name, action in self.mimic_actions.items()
            if action["spec"].keyboard_key
        }

    def _policy_index(self, mode: str) -> int | None:
        return next(
            (index for index, entry in enumerate(self.policy_entries) if entry["mode"] == mode),
            None,
        )

    def _policy_index_by_name(self, name: str) -> int | None:
        return next(
            (
                index
                for index, entry in enumerate(self.policy_entries)
                if entry["name"] == name
            ),
            None,
        )

    def _activate_policy(self, policy_index: int) -> str:
        """Change policy without discontinuously changing the commanded joints."""
        if policy_index == self.active_policy_index:
            return self.active_name

        # Keep the currently commanded target as the first point of a smooth
        # blend.  The last-action observation must describe that applied target.
        self.policy_transition_start = self.target_joint_pos.copy()
        self.policy_transition_elapsed = 0.0
        self.active_policy_index = policy_index
        self.observation_history.clear()
        self.previous_action = np.clip(
            (self.target_joint_pos - self.active_default_joint_pos) / self.ACTION_SCALE,
            -100.0,
            100.0,
        ).astype(np.float64)
        self.command[:] = 0.0
        if self.left_arm_trajectory is not None:
            self.left_arm_trajectory.reset()
        if self.active_entry["uses_mimic"]:
            motion = self.active_mimic_motion
            motion.reset()
            robot_anchor_quat = self.data.xquat[self.model.body(MIMIC_ANCHOR_BODY_NAME).id]
            motion.align_anchor_to(robot_anchor_quat)
        return self.active_name

    def _set_applied_target(self, target: np.ndarray, speed_limit: float) -> None:
        """Apply joint limits and a per-control-step joint-target speed limit."""
        target = np.clip(
            np.asarray(target, dtype=np.float64),
            self.profile.position_limits[:, 0],
            self.profile.position_limits[:, 1],
        )
        max_step = speed_limit * self.SIM_DT * self.CONTROL_DECIMATION
        target_delta = np.clip(target - self.target_joint_pos, -max_step, max_step)
        self.target_joint_pos = np.clip(
            self.target_joint_pos + target_delta,
            self.profile.position_limits[:, 0],
            self.profile.position_limits[:, 1],
        )
        self.previous_action = np.clip(
            (self.target_joint_pos - self.active_default_joint_pos) / self.ACTION_SCALE,
            -100.0,
            100.0,
        ).astype(np.float64)

    def switch_policy(self) -> str:
        """Toggle stand/walk; aborting an active Mimic returns to walk."""
        if self.mimic_in_progress:
            walk_state = self.return_to_walk()
            if walk_state is None:
                raise RuntimeError("No locomotion policy is loaded.")
            return walk_state
        target_mode = "locomotion" if self.active_mode == "stand" else "stand"
        target_index = self._policy_index(target_mode)
        if target_index is None:
            raise RuntimeError(f"No {target_mode} policy is loaded.")
        return self._activate_policy(target_index)

    def start_mimic(self, action_name: str) -> str:
        """Start one named Mimic clip from walk mode."""
        if self.mimic_in_progress:
            raise RuntimeError("Mimic playback is already active.")
        if self.active_mode != "locomotion":
            raise RuntimeError("Mimic can only start while the walk policy is active.")
        action = self.mimic_actions.get(action_name)
        if action is None:
            raise RuntimeError(f"No Mimic action named {action_name!r} is loaded.")
        return self._activate_policy(action["policy_index"])

    def return_to_walk(self) -> str | None:
        walk_index = self._policy_index("locomotion")
        if walk_index is None:
            return None
        return self._activate_policy(walk_index)

    def engage_damping_stop(self) -> None:
        if self.damping_stopped:
            return
        self.damping_stopped = True
        self.command[:] = 0.0
        print(
            f"Damping stop: inference halted, kd-only damping "
            f"(kd={self.DAMPING_STOP_KD:.1f}). Press R to reset."
        )

    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.elastic_band.set_enabled(self.elastic_band_default_enabled)
        self.data.qvel[:] = 0.0
        self.data.ctrl[:] = 0.0
        self.previous_action.fill(0.0)
        self.observation_history.clear()
        self.policy_transition_start = None
        self.policy_transition_elapsed = 0.0
        self.damping_stopped = False
        for action in self.mimic_actions.values():
            action["motion"].reset()
        if self.active_entry["uses_mimic"]:
            motion = self.active_mimic_motion
            frame = motion.frame_index
            self.data.qpos[:3] = motion.body_pos_w[frame, 0]
            self.data.qpos[3:7] = motion.body_quat_w[frame, 0]
            self.data.qpos[self.qpos_indices] = motion.joint_pos[frame]
            self.data.qvel[:3] = motion.body_lin_vel_w[frame, 0]
            self.data.qvel[3:6] = motion.body_ang_vel_w[frame, 0]
            self.data.qvel[self.qvel_indices] = motion.joint_vel[frame]
            self.target_joint_pos[:] = motion.joint_pos[frame]
        else:
            root_height = (
                self.elastic_band.suspension_height
                if self.elastic_band.enabled
                else self.profile.root_height
            )
            self.data.qpos[:3] = (0.0, 0.0, root_height)
            self.data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
            self.data.qpos[self.qpos_indices] = self.active_default_joint_pos
            self.target_joint_pos[:] = self.active_default_joint_pos
        if self.left_arm_trajectory is not None:
            self.left_arm_trajectory.reset()
        mujoco.mj_forward(self.model, self.data)
        self.elastic_band.reset_anchors(self.data)

    def _current_observation(self) -> np.ndarray:
        joint_pos = self.data.qpos[self.qpos_indices]
        joint_vel = self.data.qvel[self.qvel_indices]
        body_angular_velocity = self.data.sensor("BodyGyro").data.copy()
        if self.active_entry["uses_mimic"]:
            return self._mimic_observation()

        body_rotation = quaternion_to_rotation_matrix(self.data.qpos[3:7])
        projected_gravity = body_rotation.T @ np.asarray([0.0, 0.0, -1.0])
        observation_parts = [
            body_angular_velocity,
            projected_gravity,
            self.command,
            joint_pos - self.active_default_joint_pos,
            joint_vel,
            self.previous_action,
        ]
        if self.active_entry["uses_left_arm"]:
            observation_parts.append(self.left_arm_trajectory.observation())
        observation = np.concatenate(observation_parts)
        return np.clip(observation, -100.0, 100.0).astype(np.float32)

    def _mimic_observation(self) -> np.ndarray:
        """Build the active named Mimic policy input."""
        motion = self.active_mimic_motion
        joint_pos = self.data.qpos[self.qpos_indices]
        joint_vel = self.data.qvel[self.qvel_indices]
        body_angular_velocity = self.data.sensor("BodyGyro").data.copy()
        robot_anchor_quat = self.data.xquat[self.model.body(MIMIC_ANCHOR_BODY_NAME).id]
        relative_anchor_quat = quaternion_multiply(
            quaternion_conjugate(robot_anchor_quat),
            motion.anchor_quaternion,
        )
        relative_anchor_rotation_6d = quaternion_to_rotation_matrix(
            relative_anchor_quat
        )[:, :2].reshape(-1)
        joint_position_relative = joint_pos - self.active_default_joint_pos
        if self.active_entry["uses_mimic_history"]:
            body_linear_acceleration = self.data.sensor("BodyAcc").data.copy()
            proprio_history = motion.update_proprio_history(
                body_linear_acceleration,
                body_angular_velocity,
                joint_position_relative,
                joint_vel,
            )
            observation = np.concatenate(
                (
                    motion.command,
                    relative_anchor_rotation_6d,
                    proprio_history,
                    self.previous_action,
                )
            )
        else:
            observation = np.concatenate(
                (
                    motion.command,
                    relative_anchor_rotation_6d,
                    body_angular_velocity,
                    joint_position_relative,
                    joint_vel,
                    self.previous_action,
                )
            )
        return np.clip(observation, -100.0, 100.0).astype(np.float32)

    def update_policy(self) -> None:
        observation = self._current_observation()
        if self.active_entry["uses_mimic"]:
            policy_input = observation
        else:
            if not self.observation_history:
                for _ in range(self.HISTORY_LENGTH):
                    self.observation_history.append(observation.copy())
            else:
                self.observation_history.append(observation)
            policy_input = np.concatenate(tuple(self.observation_history))
        with torch.inference_mode():
            action = (
                self.active_entry["policy"](torch.from_numpy(policy_input).unsqueeze(0))
                .squeeze(0)
                .cpu()
                .numpy()
            )
        requested_action = np.clip(action, -100.0, 100.0).astype(np.float64)
        requested_target = np.clip(
            self.active_default_joint_pos + self.ACTION_SCALE * requested_action,
            self.profile.position_limits[:, 0],
            self.profile.position_limits[:, 1],
        )
        current_target = self.target_joint_pos.copy()
        if self.policy_transition_start is not None:
            self.policy_transition_elapsed = min(
                self.policy_transition_elapsed + self.SIM_DT * self.CONTROL_DECIMATION,
                self.POLICY_BLEND_DURATION,
            )
            u = self.policy_transition_elapsed / self.POLICY_BLEND_DURATION
            alpha = 6.0 * u**5 - 15.0 * u**4 + 10.0 * u**3
            next_target = (
                (1.0 - alpha) * self.policy_transition_start + alpha * requested_target
            )
            if self.policy_transition_elapsed >= self.POLICY_BLEND_DURATION:
                self.policy_transition_start = None
        else:
            next_target = requested_target
        if self.active_entry["uses_mimic"]:
            self.target_joint_pos[:] = current_target
            self._set_applied_target(
                next_target, self.active_mimic_action["spec"].target_speed
            )
        else:
            self.target_joint_pos = next_target
            self.previous_action = np.clip(
                (self.target_joint_pos - self.active_default_joint_pos) / self.ACTION_SCALE,
                -100.0,
                100.0,
            ).astype(np.float64)

    def prepare_physics_step(self) -> np.ndarray:
        joint_pos = self.data.qpos[self.qpos_indices]
        joint_vel = self.data.qvel[self.qvel_indices]
        if self.damping_stopped:
            # LT+B stop: kd-only damping, no position tracking or feed-forward.
            torque = -self.DAMPING_STOP_KD * joint_vel
        else:
            torque = (
                self.profile.stiffness * (self.target_joint_pos - joint_pos)
                - self.profile.damping * joint_vel
            )
        applied_torque = np.clip(
            torque, -self.profile.torque_limits, self.profile.torque_limits
        )
        self.data.ctrl[:] = 0.0
        self.data.ctrl[self.actuator_indices] = applied_torque
        self.elastic_band.apply(self.data)
        return applied_torque

    def clip_joint_velocities(self) -> None:
        self.data.qvel[self.qvel_indices] = np.clip(
            self.data.qvel[self.qvel_indices],
            -self.profile.velocity_limits,
            self.profile.velocity_limits,
        )

    def physics_step(self) -> None:
        self.prepare_physics_step()
        mujoco.mj_step(self.model, self.data)
        self.clip_joint_velocities()

    def stand(self, duration: float) -> None:
        steps = max(0, int(duration / self.SIM_DT))
        for _ in range(steps):
            self.physics_step()
        self.observation_history.clear()
        if self.left_arm_trajectory is not None:
            self.left_arm_trajectory.reset()

    def control_step(self) -> None:
        if not self.damping_stopped:
            self.update_policy()
        for _ in range(self.CONTROL_DECIMATION):
            self.physics_step()
        if self.active_entry["uses_left_arm"]:
            self.left_arm_trajectory.advance(self.SIM_DT * self.CONTROL_DECIMATION)
        # Hold the first reference frame while blending in from walk.  This
        # preserves the entire clip instead of consuming its first 0.5 s during
        # the policy hand-off.
        mimic_ready_to_advance = (
            self.active_entry["uses_mimic"]
            and self.policy_transition_start is None
        )
        motion = self.active_mimic_motion if mimic_ready_to_advance else None
        if motion is not None and motion.advance():
            completed_action = self.active_name
            walk_state = self.return_to_walk()
            if walk_state is None:
                print(
                    f"Mimic {completed_action} finished at frame {motion.frame_index}; "
                    "holding the final frame. Press R to replay."
                )
            else:
                print(
                    f"Mimic {completed_action} finished at frame {motion.frame_index}; "
                    f"entering {walk_state}."
                )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--policy",
        type=Path,
        default=None,
        help="Exported JIT policy.pt from Isaac Lab. Runs single-policy mode "
        "(for stand-leftarm or direct Mimic tests); omit to load the default "
        "walk+stand+named-Mimic policy table.",
    )
    parser.add_argument(
        "--stand-policy",
        type=Path,
        default=DEFAULT_STAND_POLICY,
        help="Stand policy for default multi-policy mode (ignored with --policy).",
    )
    parser.add_argument(
        "--walk-policy",
        type=Path,
        default=DEFAULT_WALK_POLICY,
        help="Walk policy for default multi-policy mode (ignored with --policy).",
    )
    parser.add_argument("--dof", type=int, choices=(12, 27), default=27)
    parser.add_argument(
        "--mode",
        choices=("locomotion", "stand", "mimic"),
        default="locomotion",
        help="Policy command semantics. Default multi-policy mode starts in walk; "
        "mimic is only valid with --policy for direct testing.",
    )
    parser.add_argument(
        "--motion-file",
        type=Path,
        default=None,
        help="50 Hz training motion NPZ for direct --mode mimic testing.",
    )
    parser.add_argument(
        "--mimic-start-frame",
        type=int,
        default=0,
        help="Reference start frame for direct --mode mimic testing.",
    )
    parser.add_argument(
        "--mimic-policy-target-speed",
        type=float,
        default=6.0,
        help="Joint-target speed limit for direct --mode mimic testing [rad/s].",
    )
    parser.add_argument(
        "--mimic-current-asset-defaults",
        action="store_true",
        help="Use the current Isaac shoulder-roll action/observation zero point "
        "for a directly loaded Mimic policy.",
    )
    parser.add_argument("--vx", type=float, default=0.0, help="Forward velocity command in m/s.")
    parser.add_argument("--vy", type=float, default=0.0, help="Lateral velocity command in m/s.")
    parser.add_argument("--yaw-rate", type=float, default=0.0, help="Yaw velocity command in rad/s.")
    parser.add_argument(
        "--height-command",
        type=float,
        default=0.0,
        help="Stand mode: positive crouches, negative raises the base.",
    )
    parser.add_argument(
        "--roll-command",
        type=float,
        default=0.0,
        help="Stand mode torso roll command.",
    )
    parser.add_argument(
        "--pitch-command",
        type=float,
        default=0.0,
        help="Stand mode torso pitch command.",
    )
    parser.add_argument(
        "--left-arm-traj",
        type=Path,
        default=DEFAULT_LEFT_ARM_TRAJ,
        help="Trajectory CSV for the local stand-leftarm policy.",
    )
    parser.add_argument(
        "--left-arm-start-phase",
        type=float,
        default=0.0,
        help="Trajectory phase [s] used after the 2 s safe-pose entry stage.",
    )
    arm_group = parser.add_mutually_exclusive_group()
    arm_group.add_argument(
        "--left-arm-track",
        dest="left_arm_enabled",
        action="store_true",
        help="Start a stand-leftarm policy with trajectory tracking enabled.",
    )
    arm_group.add_argument(
        "--no-left-arm-track",
        dest="left_arm_enabled",
        action="store_false",
        help="Start a stand-leftarm policy with its arm command disabled.",
    )
    parser.set_defaults(left_arm_enabled=False)
    parser.add_argument(
        "--stand-seconds",
        type=float,
        default=0.0,
        help="Fixed-pose warmup before policy control. Keep at zero for trained locomotion policies.",
    )
    parser.add_argument("--duration", type=float, default=0.0, help="Run duration; zero means until viewer closes.")
    band_group = parser.add_mutually_exclusive_group()
    band_group.add_argument(
        "--elastic-band",
        dest="elastic_band_enabled",
        action="store_true",
        help="Force-enable the shoulder elastic suspension.",
    )
    band_group.add_argument(
        "--no-elastic-band",
        dest="elastic_band_enabled",
        action="store_false",
        help="Force-disable the shoulder elastic suspension.",
    )
    parser.set_defaults(elastic_band_enabled=None)
    parser.add_argument(
        "--band-lift",
        type=float,
        default=0.0,
        help="Initial root lift above the normal standing height in meters.",
    )
    parser.add_argument("--band-anchor-height", type=float, default=3.0)
    parser.add_argument("--band-stiffness", type=float, default=500.0)
    parser.add_argument("--band-damping", type=float, default=100.0)
    parser.add_argument(
        "--band-support-ratio",
        type=float,
        default=0.3,
        help="Fraction of robot weight supported by the two shoulder bands.",
    )
    parser.add_argument(
        "--gamepad",
        action="store_true",
        help="Read policy commands from an SDL-mapped gamepad using pygame.",
    )
    parser.add_argument("--gamepad-index", type=int, default=0, help="SDL gamepad device index.")
    parser.add_argument(
        "--gamepad-deadzone",
        type=float,
        default=0.08,
        help="Normalized joystick deadzone in [0, 1).",
    )
    parser.add_argument(
        "--plot",
        "--plot-waveforms",
        dest="plot_waveforms",
        action="store_true",
        help="Open a live OpenCV telemetry waveform window.",
    )
    parser.add_argument(
        "--plot-window",
        type=float,
        default=10.0,
        help="Initial seconds of telemetry shown; adjustable from 1 to 300 in the window.",
    )
    parser.add_argument(
        "--plot-hz",
        type=float,
        default=15.0,
        help="Waveform display refresh rate; sampling remains at the 50 Hz policy rate.",
    )
    parser.add_argument(
        "--plot-joints",
        default="",
        help="Comma-separated joints for actual-position and PD-target plots. "
        "Empty uses left/right knee, left shoulder roll and left wrist yaw.",
    )
    parser.add_argument(
        "--plot-torque-joints",
        default="",
        help="Comma-separated joints for the torque plot. Empty uses all left/right "
        "hip, knee and ankle joints.",
    )
    parser.add_argument(
        "--plot-save-dir",
        type=Path,
        default=SCRIPT_DIR / "waveform_data",
        help="Directory used by the waveform Save CSV button.",
    )
    parser.add_argument(
        "--scene",
        type=Path,
        default=None,
        help="MuJoCo scene XML overriding the default scene_<dof>dof.xml. Use "
        "scene_27dof_identified.xml to match the training plant, or "
        "scene_27dof_identified_leftarm2p5kg.xml for the 2.5 kg payload tasks.",
    )
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    if args.gamepad_index < 0:
        parser.error("--gamepad-index must be non-negative")
    if not 0.0 <= args.gamepad_deadzone < 1.0:
        parser.error("--gamepad-deadzone must be in [0, 1)")
    if args.gamepad and args.headless:
        parser.error("--gamepad cannot be combined with --headless")
    if args.plot_waveforms and args.headless:
        parser.error("--plot cannot be combined with --headless")
    if not (
        TelemetryWaveform.MIN_WINDOW_SECONDS
        <= args.plot_window
        <= TelemetryWaveform.MAX_WINDOW_SECONDS
    ):
        parser.error(
            "--plot-window must be in "
            f"[{TelemetryWaveform.MIN_WINDOW_SECONDS:g}, "
            f"{TelemetryWaveform.MAX_WINDOW_SECONDS:g}]"
        )
    if args.plot_hz <= 0.0:
        parser.error("--plot-hz must be positive")
    direct_mimic = args.policy is not None and args.mode == "mimic"
    if args.mode == "mimic" and args.policy is None:
        parser.error("--mode mimic requires --policy for direct testing")
    if direct_mimic and args.motion_file is None:
        parser.error("Direct Mimic testing requires --motion-file")
    if not direct_mimic and args.motion_file is not None:
        parser.error("--motion-file requires --policy with --mode mimic")
    if args.mimic_start_frame < 0:
        parser.error("--mimic-start-frame must be non-negative")
    if args.mimic_policy_target_speed <= 0.0:
        parser.error("--mimic-policy-target-speed must be positive")
    if args.mimic_current_asset_defaults and not direct_mimic:
        parser.error("--mimic-current-asset-defaults requires --policy with --mode mimic")
    return args


def main() -> None:
    args = parse_args()
    if args.policy is not None:
        policy_name = "direct_mimic" if args.mode == "mimic" else args.mode
        policy_specs = [
            PolicySpec(
                policy_name,
                args.mode,
                args.policy.resolve(),
                use_current_asset_defaults=args.mimic_current_asset_defaults,
            )
        ]
        mimic_action_specs = []
        if args.mode == "mimic":
            mimic_action_specs.append(
                MimicActionSpec(
                    name=policy_name,
                    trigger="",
                    policy_path=args.policy.resolve(),
                    motion_path=args.motion_file.resolve(),
                    start_frame=args.mimic_start_frame,
                    target_speed=args.mimic_policy_target_speed,
                    use_current_asset_defaults=args.mimic_current_asset_defaults,
                )
            )
    else:
        policies_by_mode = {
            "stand": PolicySpec("stand", "stand", args.stand_policy.resolve()),
            "locomotion": PolicySpec("walk", "locomotion", args.walk_policy.resolve()),
        }
        mode_order = [args.mode] + [
            mode for mode in ("locomotion", "stand") if mode != args.mode
        ]
        policy_specs = [policies_by_mode[mode] for mode in mode_order]
        mimic_action_specs = list(DEFAULT_MIMIC_ACTIONS)
        policy_specs.extend(
            PolicySpec(
                spec.name,
                "mimic",
                spec.policy_path.resolve(),
                use_current_asset_defaults=spec.use_current_asset_defaults,
            )
            for spec in mimic_action_specs
        )
    initial_mode = policy_specs[0].mode

    if initial_mode == "mimic":
        command = np.zeros(3, dtype=np.float64)
    elif initial_mode == "stand":
        command = np.asarray(
            [args.height_command, args.roll_command, args.pitch_command],
            dtype=np.float64,
        )
        command = np.clip(command, (-1.0, -0.5, -0.5), (1.0, 0.5, 0.5))
    else:
        command = np.asarray([args.vx, args.vy, args.yaw_rate], dtype=np.float64)
        command = np.clip(command, (-0.6, -0.5, -1.57), (1.0, 0.5, 1.57))
    if args.gamepad:
        command.fill(0.0)
    elastic_band_enabled = args.elastic_band_enabled
    if elastic_band_enabled is None:
        elastic_band_enabled = initial_mode == "locomotion"

    simulator = HumanoidUltraSim2Sim(
        dof=args.dof,
        policy_specs=policy_specs,
        command=command,
        mimic_action_specs=mimic_action_specs,
        left_arm_traj_path=args.left_arm_traj,
        left_arm_enabled=args.left_arm_enabled,
        left_arm_start_phase=args.left_arm_start_phase,
        elastic_band_enabled=elastic_band_enabled,
        band_lift=args.band_lift,
        band_anchor_height=args.band_anchor_height,
        band_stiffness=args.band_stiffness,
        band_damping=args.band_damping,
        band_support_ratio=args.band_support_ratio,
        scene_path=args.scene,
    )
    simulator.stand(args.stand_seconds)
    gamepad = (
        GamepadCommandSource(
            args.gamepad_index,
            args.gamepad_deadzone,
            simulator.mimic_trigger_map,
        )
        if args.gamepad
        else None
    )
    if simulator.left_arm_trajectory is not None:
        state = "ON" if simulator.left_arm_trajectory.enabled else "OFF"
        print(f"Left-arm trajectory: {state} (L toggles tracking).")
    loaded_names = "/".join(entry["name"] for entry in simulator.policy_entries)
    if initial_mode == "mimic":
        motion = simulator.active_mimic_motion
        print(
            f"Loaded {args.dof}-DOF Mimic policy: {loaded_names}. Reference: "
            f"{motion.path} ({motion.frame_count} frames, "
            f"{motion.fps:g} Hz, start={motion.start_frame})."
        )
    elif initial_mode == "stand":
        print(
            f"Loaded {args.dof}-DOF policies: {loaded_names} (active: "
            f"{simulator.active_name}). Command: "
            f"height={command[0]:.2f}, roll={command[1]:.2f}, pitch={command[2]:.2f}"
        )
    else:
        print(
            f"Loaded {args.dof}-DOF policies: {loaded_names} (active: "
            f"{simulator.active_name}). Command: "
            f"vx={command[0]:.2f}, vy={command[1]:.2f}, yaw={command[2]:.2f}"
        )
    if simulator.mimic_actions and initial_mode != "mimic":
        for name, action in simulator.mimic_actions.items():
            motion = action["motion"]
            spec = action["spec"]
            print(
                f"Mimic {name} ready: {motion.path} "
                f"({motion.frame_count} frames, {motion.fps:g} Hz, "
                f"trigger={GamepadCommandSource.trigger_label(spec.trigger)}/"
                f"{spec.keyboard_key.upper()})."
            )

    start_time = time.perf_counter()
    control_dt = simulator.SIM_DT * simulator.CONTROL_DECIMATION

    if args.headless:
        if args.duration > 0.0:
            duration = args.duration
        elif simulator.active_entry["uses_mimic"]:
            duration = simulator.active_mimic_motion.duration
        else:
            duration = 10.0
        while simulator.data.time < args.stand_seconds + duration:
            simulator.control_step()
        print(
            f"Finished headless rollout: t={simulator.data.time:.3f}s, "
            f"base_z={simulator.data.qpos[2]:.3f}m"
        )
        return

    import mujoco.viewer

    waveform = None
    glfw_key_right = 262
    glfw_key_left = 263
    glfw_key_down = 264
    glfw_key_up = 265

    def print_status() -> None:
        band_status = "ON" if simulator.elastic_band.enabled else "OFF"
        arm_status = (
            "N/A"
            if simulator.left_arm_trajectory is None
            else "RETURNING"
            if simulator.left_arm_trajectory.returning
            else "ON"
            if simulator.left_arm_trajectory.enabled
            else "OFF"
        )
        if simulator.active_mode == "mimic":
            motion = simulator.active_mimic_motion
            command_status = (
                f"reference={motion.frame_index + 1}/{motion.frame_count} "
                f"({motion.frame_index / motion.fps:.2f}s)"
            )
        elif simulator.active_mode == "stand":
            command_status = (
                f"height={simulator.command[0]:.2f}, "
                f"roll={simulator.command[1]:.2f}, pitch={simulator.command[2]:.2f}"
            )
        else:
            command_status = (
                f"vx={simulator.command[0]:.2f}, "
                f"vy={simulator.command[1]:.2f}, yaw={simulator.command[2]:.2f}"
            )
        if gamepad is None:
            gamepad_status = "OFF"
        elif not gamepad.connected:
            gamepad_status = "DISCONNECTED"
        else:
            gamepad_status = "ON" if gamepad.enabled else "DISABLED"
        damping_status = " [DAMPING STOP]" if simulator.damping_stopped else ""
        print(
            f"state={simulator.control_state}, policy={simulator.active_name}"
            f"{damping_status} | command {command_status} | "
            f"band={band_status}, length={simulator.elastic_band.length:.2f}m, "
            f"left_arm={arm_status}, gamepad={gamepad_status}"
        )

    def toggle_left_arm() -> None:
        if simulator.left_arm_trajectory is None or not simulator.active_entry["uses_left_arm"]:
            print("The active policy has no left-arm trajectory observation.")
        else:
            simulator.left_arm_trajectory.toggle()

    def switch_policy() -> None:
        if len(simulator.policy_entries) < 2:
            print(f"Policy switch ignored: only the {simulator.active_name} policy is loaded.")
        elif simulator.damping_stopped:
            print("Cannot switch policy during damping stop; press R to reset.")
        else:
            print(f"Active policy switched to: {simulator.switch_policy()}")

    def start_mimic(action_name: str) -> None:
        if action_name not in simulator.mimic_actions:
            print(f"Mimic trigger ignored: action {action_name!r} is not loaded.")
        elif simulator.damping_stopped:
            print("Cannot start Mimic during damping stop; press R to reset.")
        elif simulator.mimic_in_progress:
            print(f"Mimic trigger ignored: {simulator.control_state} is already active.")
        elif simulator.active_mode != "locomotion":
            print("Mimic can only start from walk mode; press P to switch to walk first.")
        else:
            try:
                state = simulator.start_mimic(action_name)
            except RuntimeError as exc:
                print(f"Mimic trigger ignored: {exc}")
            else:
                print(f"Active policy switched to: {state}")

    def key_callback(keycode: int) -> None:
        handled = True
        keyboard_key = chr(keycode).lower() if 0 <= keycode < 128 else ""
        if keycode in (ord("R"), ord("r")):
            simulator.reset()
            simulator.stand(args.stand_seconds)
        elif keycode in (ord("X"), ord("x"), 32):
            simulator.command[:] = 0.0
        elif keycode in (ord("P"), ord("p")):
            switch_policy()
        elif keyboard_key in simulator.mimic_keyboard_map:
            start_mimic(simulator.mimic_keyboard_map[keyboard_key])
        elif keycode in (ord("0"),):
            simulator.engage_damping_stop()
        elif simulator.mimic_in_progress and keycode in (
            ord("W"),
            ord("w"),
            ord("S"),
            ord("s"),
            ord("A"),
            ord("a"),
            ord("D"),
            ord("d"),
            ord("Q"),
            ord("q"),
            ord("E"),
            ord("e"),
            glfw_key_up,
            glfw_key_down,
            glfw_key_left,
            glfw_key_right,
        ):
            print("Mimic mode follows the NPZ reference; movement commands are disabled.")
        elif keycode in (ord("W"), ord("w"), glfw_key_up):
            simulator.command[0] = min(1.0, simulator.command[0] + 0.1)
        elif keycode in (ord("S"), ord("s"), glfw_key_down):
            lower_bound = -1.0 if simulator.active_mode == "stand" else -0.6
            simulator.command[0] = max(lower_bound, simulator.command[0] - 0.1)
        elif keycode in (ord("A"), ord("a")):
            simulator.command[1] = min(0.5, simulator.command[1] + 0.1)
        elif keycode in (ord("D"), ord("d")):
            simulator.command[1] = max(-0.5, simulator.command[1] - 0.1)
        elif keycode in (ord("Q"), ord("q"), glfw_key_left):
            upper_bound = 0.5 if simulator.active_mode == "stand" else 1.57
            simulator.command[2] = min(upper_bound, simulator.command[2] + 0.1)
        elif keycode in (ord("E"), ord("e"), glfw_key_right):
            lower_bound = -0.5 if simulator.active_mode == "stand" else -1.57
            simulator.command[2] = max(lower_bound, simulator.command[2] - 0.1)
        elif keycode in (ord("7"),):
            simulator.elastic_band.adjust_length(-0.1)
        elif keycode in (ord("8"),):
            simulator.elastic_band.adjust_length(0.1)
        elif keycode in (ord("9"), ord("B"), ord("b")):
            simulator.elastic_band.toggle()
        elif keycode in (ord("L"), ord("l")):
            toggle_left_arm()
        else:
            handled = False
        if handled:
            print_status()

    print("Click the MuJoCo window first so it can receive keyboard input.")
    if initial_mode == "mimic":
        print("Mimic: the NPZ reference advances at 50 Hz; R restarts from the selected frame.")
    elif initial_mode == "stand":
        print("Stand: W crouch, S raise, A/D roll, Q/E pitch.")
    else:
        print("Move: W/S or Up/Down, A/D lateral, Q/E or Left/Right yaw.")
    print("Band: 7 shorter/raise, 8 longer/lower, 9/B release or attach.")
    if simulator.left_arm_trajectory is not None:
        print("Left arm: L toggles trajectory tracking or returns to the default pose.")
    if len(simulator.policy_entries) > 1:
        print("Policy: P switches stand/walk.")
    if simulator.mimic_keyboard_map:
        keyboard_bindings = ", ".join(
            f"{key.upper()}={name.replace('_', '-')}"
            for key, name in simulator.mimic_keyboard_map.items()
        )
        print(
            f"Mimic: {keyboard_bindings} from walk; each returns to walk automatically."
        )
    print("Other: X/Space stop, 0 damping stop, R reset and restore the default band state.")
    if gamepad is not None:
        if initial_mode == "stand":
            print("Gamepad: left Y=height, left X=roll, right X=pitch.")
        else:
            print("Gamepad: left Y=vx, left X=vy, right X=yaw rate.")
        print(
            "Gamepad buttons: Y toggles control; X switches the stand/walk policy; "
            "A toggles the stand-leftarm trajectory."
        )
        if simulator.mimic_trigger_map:
            gamepad_bindings = ", ".join(
                f"{GamepadCommandSource.trigger_label(trigger)}={name.replace('_', '-')}"
                for trigger, name in simulator.mimic_trigger_map.items()
            )
            print(f"Gamepad Mimic: {gamepad_bindings}.")
        print("Gamepad safety: LT+B damping stop; LB+RB clears the command and disables control.")
    print_status()
    try:
        with mujoco.viewer.launch_passive(
            simulator.model, simulator.data, key_callback=key_callback
        ) as viewer:
            if args.plot_waveforms:
                plot_joints = tuple(
                    name.strip()
                    for name in args.plot_joints.split(",")
                    if name.strip()
                )
                plot_torque_joints = tuple(
                    name.strip()
                    for name in args.plot_torque_joints.split(",")
                    if name.strip()
                )
                waveform = TelemetryWaveform(
                    simulator,
                    window_seconds=args.plot_window,
                    refresh_hz=args.plot_hz,
                    joint_names=plot_joints,
                    torque_joint_names=plot_torque_joints,
                    save_directory=args.plot_save_dir,
                )
                waveform.sample(simulator)
                print(
                    "Live waveforms: click Window -/+, Pause or Save CSV; keyboard "
                    "[-]/[+], Space and S provide the same controls. Esc closes "
                    "only the waveform window."
                )
            while viewer.is_running():
                step_start = time.perf_counter()
                if gamepad is not None:
                    gamepad_command = gamepad.poll(simulator.active_mode)
                    if not simulator.damping_stopped and simulator.accepts_motion_commands:
                        simulator.command[:] = gamepad_command
                    if gamepad.damping_stop_requested:
                        simulator.engage_damping_stop()
                        print_status()
                    if gamepad.x_toggle_requested:
                        switch_policy()
                        print_status()
                    if gamepad.mimic_requested is not None:
                        start_mimic(gamepad.mimic_requested)
                        print_status()
                    if gamepad.a_toggle_requested:
                        toggle_left_arm()
                        print_status()
                simulator.control_step()
                if waveform is not None:
                    waveform.sample(simulator)
                viewer.sync()
                if args.duration > 0.0 and time.perf_counter() - start_time >= args.duration:
                    break
                sleep_time = control_dt - (time.perf_counter() - step_start)
                if sleep_time > 0.0:
                    time.sleep(sleep_time)
    finally:
        if waveform is not None:
            waveform.close()
        if gamepad is not None:
            gamepad.close()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Run Humanoid Ultra Isaac Lab policies in MuJoCo.

Without --policy this loads the default stand and walk policies together:
gamepad X (or keyboard P) switches between them, and gamepad LT+B (or
keyboard 0) stops inference and latches kd-only damping until reset (R).

Use ``--mimic-policy`` with the training motion ``.npz`` to add a one-shot
walk-to-Mimic-to-walk action, or ``--mode mimic --policy`` for direct testing.
The reference is advanced once per 50 Hz policy step.
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


DEFAULT_STAND_POLICY = Path(
    "/home/zxh/unitree_rl_lab/logs/rsl_rl/humanoidultra27dof_stand_leftarm/"
    "2026-06-30_19-24-09/exported/policy.pt"
)
DEFAULT_WALK_POLICY = Path(
    "/home/zxh/unitree_rl_lab/logs/rsl_rl/humanoidultra27dof_flat/"
    "2026-06-13_12-19-18/exported/policy.pt"
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

TRAINING_POSITION_LIMITS = {
    "left_hip_roll_joint": (-0.25, 1.57),
    "right_hip_roll_joint": (-1.57, 0.25),
    "left_hip_yaw_joint": (-1.57, 1.57),
    "right_hip_yaw_joint": (-1.57, 1.57),
    "left_hip_pitch_joint": (-1.57, 1.57),
    "right_hip_pitch_joint": (-1.57, 1.57),
    "left_knee_joint": (0.0, 2.36),
    "right_knee_joint": (0.0, 2.36),
    "left_ankle_pitch_joint": (-0.7, 0.95),
    "right_ankle_pitch_joint": (-0.7, 0.95),
    "left_ankle_roll_joint": (-0.45, 0.45),
    "right_ankle_roll_joint": (-0.45, 0.45),
    "waist_yaw_joint": (-2.62, 2.62),
    "left_shoulder_pitch_joint": (-2.4, 1.2),
    "right_shoulder_pitch_joint": (-1.2, 2.4),
    "left_shoulder_roll_joint": (-0.6, 2.7),
    "right_shoulder_roll_joint": (-2.7, 0.6),
    "left_shoulder_yaw_joint": (-3.7175512, 1.623156),
    "right_shoulder_yaw_joint": (-1.623156, 3.7175512),
    "left_elbow_joint": (-2.17, 0.0),
    "right_elbow_joint": (0.0, 2.17),
    "left_wrist_yaw_joint": (-1.1519173, 4.043510),
    "right_wrist_yaw_joint": (-4.043510, 1.1519173),
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


class GamepadCommandSource:
    """Poll an SDL-mapped gamepad without blocking the MuJoCo control loop."""

    AXIS_MAX = 32768.0
    RECONNECT_PERIOD = 1.0

    def __init__(self, index: int, deadzone: float):
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
        self._previous_mimic = False
        self._previous_stop = False
        self._previous_damping = False
        self.x_toggle_requested = False
        self.a_toggle_requested = False
        self.mimic_requested = False
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
        self._previous_mimic = False
        self._previous_stop = False
        self._previous_damping = False
        self.x_toggle_requested = False
        self.a_toggle_requested = False
        self.mimic_requested = False
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
        self._previous_mimic = False
        self._previous_stop = False
        self._previous_damping = False
        self.x_toggle_requested = False
        self.a_toggle_requested = False
        self.mimic_requested = False
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
        self.mimic_requested = False
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
            mimic_pressed = bool(
                lt_pressed
                and self._controller.get_button(self._pygame.CONTROLLER_BUTTON_DPAD_DOWN)
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
            if (
                mimic_pressed
                and not self._previous_mimic
                and not damping_pressed
                and not stop_pressed
                and self.enabled
            ):
                self.mimic_requested = True
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
            [TRAINING_POSITION_LIMITS[name] for name in names], dtype=np.float64
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
    """Reproduce the 15-D left-arm command used by the Isaac Lab task."""

    def __init__(
        self,
        traj_path: Path,
        profile: RobotProfile,
        enabled: bool,
        period: float = 6.0,
        blend_time: float = 2.0,
        ref_vel_scale: float = 0.25,
        start_phase: float = 1.5,
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
            [profile.default_joint_pos[joint_index[name]] for name in LEFT_ARM_JOINTS],
            dtype=np.float64,
        )
        self.period = period
        self.blend_time = blend_time
        self.ref_vel_scale = ref_vel_scale
        # Blending sweeps the arm along a straight joint-space path that is not
        # collision-aware; starting near phase 0 (or 4.5s+) drives the wrist
        # through the hip.  Offsetting the start phase keeps the blend clear.
        self.start_phase = start_phase
        self.default_enabled = enabled
        self.enabled = enabled
        self.elapsed = 0.0

    def reset(self) -> None:
        self.enabled = self.default_enabled
        self.elapsed = 0.0

    def toggle(self) -> None:
        self.enabled = not self.enabled
        if self.enabled:
            self.elapsed = 0.0

    def advance(self, dt: float) -> None:
        self.elapsed += dt

    def observation(self) -> np.ndarray:
        if not self.enabled:
            return np.zeros(LEFT_ARM_COMMAND_DIM, dtype=np.float32)

        phase = (self.start_phase + self.elapsed) % self.period
        angles = phase * self.omega
        cosines = np.cos(angles)
        sines = np.sin(angles)
        q_traj = cosines @ self.cos_coeff + sines @ self.sin_coeff
        dq_traj = (
            -(sines * self.omega) @ self.cos_coeff
            + (cosines * self.omega) @ self.sin_coeff
        )
        delta = q_traj - self.default_q

        if self.blend_time > 0.0:
            u = np.clip(self.elapsed / self.blend_time, 0.0, 1.0)
            alpha = 6.0 * u**5 - 15.0 * u**4 + 10.0 * u**3
            alpha_dot = (
                30.0 * u**4 - 60.0 * u**3 + 30.0 * u**2
            ) / self.blend_time
        else:
            alpha = 1.0
            alpha_dot = 0.0

        q_ref_rel = alpha * delta
        dq_ref = alpha_dot * delta + alpha * dq_traj
        return np.concatenate(
            (q_ref_rel, dq_ref * self.ref_vel_scale, np.ones(1))
        ).astype(np.float32)


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
    MIMIC_SUPPORT_RELEASE_DURATION = 1.0
    # Uniform joint damping used by the LT+B kd-only stop.
    DAMPING_STOP_KD = 5.0

    def __init__(
        self,
        dof: int,
        policy_specs: list[PolicySpec],
        command: np.ndarray,
        mimic_motion_path: Path | None,
        mimic_start_frame: int,
        left_arm_traj_path: Path | None,
        left_arm_enabled: bool,
        left_arm_start_phase: float,
        elastic_band_enabled: bool,
        band_lift: float,
        band_anchor_height: float,
        band_stiffness: float,
        band_damping: float,
        band_support_ratio: float,
        mimic_prepare_time: float = 0.0,
        mimic_recover_time: float = 0.0,
        mimic_transition_timeout: float = 2.0,
        mimic_transition_target_speed: float = 1.0,
        mimic_policy_target_speed: float = 6.0,
        mimic_ready_rms: float = 0.12,
        mimic_ready_max_error: float = 0.25,
        mimic_ready_max_velocity: float = 0.8,
    ):
        self.profile = make_profile(dof)
        repository_root = Path(__file__).resolve().parents[2]
        model_path = (
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
            uses_mimic = input_dim == self.profile.mimic_observation_dim
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
                        f"policy must have {self.profile.mimic_observation_dim} inputs."
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
                        f"or {self.profile.mimic_observation_dim} total for Mimic."
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
            self.policy_entries.append(
                {
                    "name": spec.name,
                    "mode": spec.mode,
                    "policy": policy,
                    "uses_left_arm": uses_left_arm,
                    "uses_mimic": uses_mimic,
                }
            )
        self.active_policy_index = 0

        if any(entry["uses_mimic"] for entry in self.policy_entries):
            if mimic_motion_path is None:
                raise ValueError("Mimic mode requires --motion-file /path/to/motion.npz.")
            self.mimic_motion = MimicMotion(
                mimic_motion_path.resolve(), self.profile, mimic_start_frame
            )
        else:
            self.mimic_motion = None

        if any(entry["uses_left_arm"] for entry in self.policy_entries):
            trajectory_path = left_arm_traj_path or Path(__file__).with_name(
                "left_wrist_pitch_traj.csv"
            )
            self.left_arm_trajectory = LeftArmTrajectory(
                trajectory_path.resolve(),
                self.profile,
                left_arm_enabled,
                start_phase=left_arm_start_phase,
            )
        else:
            self.left_arm_trajectory = None
        self.command = command.astype(np.float64)
        transition_values = (
            mimic_prepare_time,
            mimic_recover_time,
            mimic_transition_timeout,
            mimic_ready_rms,
            mimic_ready_max_error,
            mimic_ready_max_velocity,
        )
        if any(value < 0.0 for value in transition_values):
            raise ValueError("Mimic transition durations and tolerances must be non-negative.")
        if mimic_transition_target_speed <= 0.0 or mimic_policy_target_speed <= 0.0:
            raise ValueError("Mimic target speed limits must be positive.")
        self.mimic_prepare_time = float(mimic_prepare_time)
        self.mimic_recover_time = float(mimic_recover_time)
        self.mimic_transition_timeout = float(mimic_transition_timeout)
        self.mimic_transition_target_speed = float(mimic_transition_target_speed)
        self.mimic_policy_target_speed = float(mimic_policy_target_speed)
        self.mimic_ready_rms = float(mimic_ready_rms)
        self.mimic_ready_max_error = float(mimic_ready_max_error)
        self.mimic_ready_max_velocity = float(mimic_ready_max_velocity)
        self.previous_action = np.zeros(self.profile.dof, dtype=np.float64)
        self.target_joint_pos = self.profile.default_joint_pos.copy()
        self.observation_history: deque[np.ndarray] = deque(maxlen=self.HISTORY_LENGTH)
        self.policy_transition_start: np.ndarray | None = None
        self.policy_transition_elapsed = 0.0
        self.mimic_transition_state: str | None = None
        self.mimic_transition_start = self.profile.default_joint_pos.copy()
        self.mimic_transition_goal = self.profile.default_joint_pos.copy()
        self.mimic_transition_elapsed = 0.0
        self.mimic_transition_duration = 0.0
        self.mimic_transition_wait_reported = False
        self.mimic_recover_releasing = False
        self.mimic_transition_root_pos = np.zeros(3, dtype=np.float64)
        self.mimic_transition_root_quat = np.asarray(
            (1.0, 0.0, 0.0, 0.0), dtype=np.float64
        )
        self.mimic_transition_root_lin_vel = np.zeros(3, dtype=np.float64)
        self.mimic_transition_root_ang_vel = np.zeros(3, dtype=np.float64)
        self.mimic_transition_body_id = self.model.body("base_link").id
        self.mimic_transition_body_mass = float(np.sum(self.model.body_mass))
        self._mimic_support_force = np.zeros(3, dtype=np.float64)
        self._mimic_support_torque = np.zeros(3, dtype=np.float64)
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
    def control_state(self) -> str:
        """Human-readable state, including the non-policy Mimic hand-off stages."""
        if self.mimic_transition_state is not None:
            return self.mimic_transition_state
        if self.active_entry["uses_mimic"]:
            return "pick_play"
        return self.active_name

    @property
    def mimic_in_progress(self) -> bool:
        return self.mimic_transition_state is not None or self.active_entry["uses_mimic"]

    @property
    def accepts_motion_commands(self) -> bool:
        return not self.mimic_in_progress and self.active_mode in ("stand", "locomotion")

    @property
    def has_mimic_policy(self) -> bool:
        return any(entry["uses_mimic"] for entry in self.policy_entries)

    def _policy_index(self, mode: str) -> int | None:
        return next(
            (index for index, entry in enumerate(self.policy_entries) if entry["mode"] == mode),
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
        self.mimic_transition_state = None
        self.active_policy_index = policy_index
        self.observation_history.clear()
        self.previous_action = np.clip(
            (self.target_joint_pos - self.profile.default_joint_pos) / self.ACTION_SCALE,
            -100.0,
            100.0,
        ).astype(np.float64)
        self.command[:] = 0.0
        if self.left_arm_trajectory is not None:
            self.left_arm_trajectory.reset()
        if self.active_entry["uses_mimic"]:
            self.mimic_motion.reset()
            robot_anchor_quat = self.data.xquat[self.model.body(MIMIC_ANCHOR_BODY_NAME).id]
            self.mimic_motion.align_anchor_to(robot_anchor_quat)
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
            (self.target_joint_pos - self.profile.default_joint_pos) / self.ACTION_SCALE,
            -100.0,
            100.0,
        ).astype(np.float64)

    def _begin_mimic_joint_transition(
        self, state: str, goal: np.ndarray, duration: float
    ) -> str:
        self.mimic_transition_state = state
        self.mimic_transition_start = self.target_joint_pos.copy()
        self.mimic_transition_goal = np.clip(
            np.asarray(goal, dtype=np.float64),
            self.profile.position_limits[:, 0],
            self.profile.position_limits[:, 1],
        )
        self.mimic_transition_elapsed = 0.0
        self.mimic_transition_duration = duration
        self.mimic_transition_wait_reported = False
        self.mimic_recover_releasing = False
        self.policy_transition_start = None
        self.policy_transition_elapsed = 0.0
        self.observation_history.clear()
        self.command[:] = 0.0
        return self.control_state

    def _begin_mimic_prepare(self) -> str:
        self.mimic_motion.reset()
        robot_anchor_quat = self.data.xquat[self.model.body(MIMIC_ANCHOR_BODY_NAME).id]
        self.mimic_motion.align_anchor_to(robot_anchor_quat)
        goal = self.mimic_motion.joint_pos[self.mimic_motion.start_frame]
        state = self._begin_mimic_joint_transition(
            "pick_prepare", goal, self.mimic_prepare_time
        )
        frame = self.mimic_motion.start_frame
        alignment_rotation = quaternion_to_rotation_matrix(
            self.mimic_motion.anchor_alignment
        )
        self.mimic_transition_root_pos[:] = self.data.qpos[:3]
        self.mimic_transition_root_pos[2] = self.mimic_motion.body_pos_w[frame, 0, 2]
        self.mimic_transition_root_quat[:] = quaternion_multiply(
            self.mimic_motion.anchor_alignment,
            self.mimic_motion.body_quat_w[frame, 0],
        )
        self.mimic_transition_root_lin_vel[:] = (
            alignment_rotation @ self.mimic_motion.body_lin_vel_w[frame, 0]
        )
        self.mimic_transition_root_ang_vel[:] = (
            alignment_rotation @ self.mimic_motion.body_ang_vel_w[frame, 0]
        )
        print(
            f"Mimic prepare started: {self.mimic_prepare_time:.2f}s joint trajectory "
            f"to reference frame {self.mimic_motion.start_frame}."
        )
        return state

    def _begin_mimic_recover(self) -> str | None:
        walk_index = self._policy_index("locomotion")
        if walk_index is None:
            return None
        if self.mimic_transition_state == "pick_recover":
            return self.control_state
        state = self._begin_mimic_joint_transition(
            "pick_recover", self.profile.default_joint_pos, self.mimic_recover_time
        )
        root_rotation = quaternion_to_rotation_matrix(self.data.qpos[3:7])
        root_yaw = float(np.arctan2(root_rotation[1, 0], root_rotation[0, 0]))
        self.mimic_transition_root_pos[:] = self.data.qpos[:3]
        self.mimic_transition_root_pos[2] = self.profile.root_height
        self.mimic_transition_root_quat[:] = (
            np.cos(0.5 * root_yaw),
            0.0,
            0.0,
            np.sin(0.5 * root_yaw),
        )
        self.mimic_transition_root_lin_vel.fill(0.0)
        self.mimic_transition_root_ang_vel.fill(0.0)
        print(
            f"Mimic recover started: {self.mimic_recover_time:.2f}s joint trajectory "
            "to the walk-ready pose."
        )
        return state

    def _update_mimic_joint_transition(self) -> None:
        dt = self.SIM_DT * self.CONTROL_DECIMATION
        self.mimic_transition_elapsed += dt
        if self.mimic_recover_releasing:
            previous_target = self.target_joint_pos.copy()
            self.update_policy()
            walk_target = self.target_joint_pos.copy()
            self.target_joint_pos[:] = previous_target
            self._set_applied_target(
                walk_target, self.mimic_transition_target_speed
            )
            return
        u = np.clip(
            self.mimic_transition_elapsed / self.mimic_transition_duration,
            0.0,
            1.0,
        )
        alpha = 6.0 * u**5 - 15.0 * u**4 + 10.0 * u**3
        if self.mimic_transition_state == "pick_prepare":
            previous_target = self.target_joint_pos.copy()
            self.update_policy()
            balance_target = self.target_joint_pos.copy()
            self.target_joint_pos[:] = previous_target
            # Keep walk feedback underneath a fixed reference-pose trajectory.
            # A temporary root stabilizer supplies the missing balance support
            # while the walk contribution fades out.
            desired_target = (
                (1.0 - alpha) * balance_target
                + alpha * self.mimic_transition_goal
            )
        else:
            # First reach the known walk-ready pose under root support.  Walk
            # policy inference starts only after this fixed trajectory passes
            # the readiness gate.
            desired_target = (
                (1.0 - alpha) * self.mimic_transition_start
                + alpha * self.mimic_transition_goal
            )
        self._set_applied_target(desired_target, self.mimic_transition_target_speed)

    def _mimic_transition_is_ready(self) -> tuple[bool, float, float, float]:
        joint_pos = self.data.qpos[self.qpos_indices]
        joint_vel = self.data.qvel[self.qvel_indices]
        # Prepare must enter the Pick policy while the robot is both upright
        # and inside the policy's reference-state neighborhood; it need not
        # wait for the nominal trajectory clock if that gate is reached early.
        # Recover instead gates against the live walk target being applied.
        readiness_target = (
            self.mimic_transition_goal
            if self.mimic_transition_state == "pick_prepare"
            else self.target_joint_pos
        )
        position_error = joint_pos - readiness_target
        rms_error = float(np.sqrt(np.mean(position_error**2)))
        max_error = float(np.max(np.abs(position_error)))
        max_velocity = float(np.max(np.abs(joint_vel)))
        root_up = float(quaternion_to_rotation_matrix(self.data.qpos[3:7])[2, 2])
        trunk_id = self.model.body(MIMIC_ANCHOR_BODY_NAME).id
        trunk_up = float(self.data.xmat[trunk_id].reshape(3, 3)[2, 2])
        balanced = self.data.qpos[2] > 0.65 and root_up > 0.7 and trunk_up > 0.7
        ready = (
            balanced
            and rms_error <= self.mimic_ready_rms
            and max_error <= self.mimic_ready_max_error
            and max_velocity <= self.mimic_ready_max_velocity
        )
        return ready, rms_error, max_error, max_velocity

    def _finish_mimic_joint_transition_if_ready(self) -> None:
        if self.mimic_transition_state is None:
            return
        if self.mimic_recover_releasing:
            if self.mimic_transition_elapsed < self.mimic_transition_duration:
                return
            ready, rms_error, max_error, max_velocity = self._mimic_transition_is_ready()
            if ready:
                self.mimic_transition_state = None
                self.mimic_recover_releasing = False
                print(
                    "Mimic recover complete: position RMS={:.3f}rad, max={:.3f}rad, "
                    "max velocity={:.3f}rad/s; walk policy restored.".format(
                        rms_error, max_error, max_velocity
                    )
                )
            return
        if (
            self.mimic_transition_state == "pick_recover"
            and self.mimic_transition_elapsed < self.mimic_transition_duration
        ):
            return
        ready, rms_error, max_error, max_velocity = self._mimic_transition_is_ready()
        if ready:
            completed_state = self.mimic_transition_state
            if completed_state == "pick_prepare":
                mimic_index = self._policy_index("mimic")
                if mimic_index is None:
                    raise RuntimeError("No Mimic policy is loaded.")
                self._activate_policy(mimic_index)
                print(
                    "Mimic prepare complete: position RMS={:.3f}rad, max={:.3f}rad, "
                    "max velocity={:.3f}rad/s; starting Pick playback.".format(
                        rms_error, max_error, max_velocity
                    )
                )
            else:
                walk_index = self._policy_index("locomotion")
                if walk_index is None:
                    raise RuntimeError("No walk policy is loaded.")
                self._activate_policy(walk_index)
                self.mimic_transition_state = "pick_recover"
                self.mimic_recover_releasing = True
                self.mimic_transition_elapsed = 0.0
                self.mimic_transition_duration = self.MIMIC_SUPPORT_RELEASE_DURATION
                self.mimic_transition_wait_reported = False
                print(
                    "Mimic recover pose reached: position RMS={:.3f}rad, max={:.3f}rad, "
                    "max velocity={:.3f}rad/s; releasing support into walk.".format(
                        rms_error, max_error, max_velocity
                    )
                )
            return

        wait_time = self.mimic_transition_elapsed - self.mimic_transition_duration
        if (
            wait_time >= self.mimic_transition_timeout
            and not self.mimic_transition_wait_reported
        ):
            self.mimic_transition_wait_reported = True
            print(
                "{} is holding its fixed goal: readiness gate not met after {:.2f}s "
                "(position RMS={:.3f}rad, max={:.3f}rad, max velocity={:.3f}rad/s).".format(
                    self.mimic_transition_state,
                    wait_time,
                    rms_error,
                    max_error,
                    max_velocity,
                )
            )

    def _apply_mimic_transition_support(self) -> None:
        """Apply a temporary 6-D root stabilizer during Sim2Sim hand-offs."""
        support_active = self.mimic_transition_state is not None or (
            self.active_entry["uses_mimic"]
            and self.policy_transition_start is not None
            and self.mimic_prepare_time > 0.0
        )
        if not support_active:
            return

        support_scale = 1.0
        if self.mimic_transition_state == "pick_recover" and self.mimic_recover_releasing:
            u = np.clip(
                self.mimic_transition_elapsed / self.mimic_transition_duration,
                0.0,
                1.0,
            )
            support_scale = 1.0 - (6.0 * u**5 - 15.0 * u**4 + 10.0 * u**3)
        elif self.active_entry["uses_mimic"] and self.policy_transition_start is not None:
            u = np.clip(
                self.policy_transition_elapsed / self.POLICY_BLEND_DURATION,
                0.0,
                1.0,
            )
            support_scale = 1.0 - (6.0 * u**5 - 15.0 * u**4 + 10.0 * u**3)

        root_pos = self.data.qpos[:3]
        root_quat = self.data.qpos[3:7]
        root_lin_vel = self.data.qvel[:3]
        root_ang_vel = self.data.qvel[3:6]
        position_error = self.mimic_transition_root_pos - root_pos
        velocity_error = self.mimic_transition_root_lin_vel - root_lin_vel
        self._mimic_support_force[:] = (
            900.0 * position_error
            + 180.0 * velocity_error
            + np.asarray((0.0, 0.0, self.mimic_transition_body_mass * 9.81))
        )
        max_force = 2.0 * self.mimic_transition_body_mass * 9.81
        self._mimic_support_force[:] = np.clip(
            self._mimic_support_force, -max_force, max_force
        )
        self._mimic_support_force *= support_scale

        orientation_error = quaternion_multiply(
            self.mimic_transition_root_quat,
            quaternion_conjugate(root_quat),
        )
        if orientation_error[0] < 0.0:
            orientation_error *= -1.0
        rotation_error = 2.0 * orientation_error[1:]
        self._mimic_support_torque[:] = (
            350.0 * rotation_error
            + 70.0 * (self.mimic_transition_root_ang_vel - root_ang_vel)
        )
        self._mimic_support_torque[:] = np.clip(
            self._mimic_support_torque, -250.0, 250.0
        )
        self._mimic_support_torque *= support_scale
        mujoco.mj_applyFT(
            self.model,
            self.data,
            self._mimic_support_force,
            self._mimic_support_torque,
            self.data.xpos[self.mimic_transition_body_id],
            self.mimic_transition_body_id,
            self.data.qfrc_applied,
        )

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

    def start_mimic(self) -> str:
        """Start the one-shot Mimic clip from walk mode."""
        if self.mimic_in_progress:
            raise RuntimeError("Mimic playback or transition is already active.")
        if self.active_mode != "locomotion":
            raise RuntimeError("Mimic can only start while the walk policy is active.")
        mimic_index = self._policy_index("mimic")
        if mimic_index is None:
            raise RuntimeError("No Mimic policy is loaded.")
        if self.mimic_prepare_time > 0.0:
            return self._begin_mimic_prepare()
        return self._activate_policy(mimic_index)

    def return_to_walk(self) -> str | None:
        walk_index = self._policy_index("locomotion")
        if walk_index is None:
            return None
        if self.mimic_in_progress and self.mimic_recover_time > 0.0:
            return self._begin_mimic_recover()
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
        self.mimic_transition_state = None
        self.mimic_transition_elapsed = 0.0
        self.mimic_transition_wait_reported = False
        self.mimic_recover_releasing = False
        self.damping_stopped = False
        if self.mimic_motion is not None:
            self.mimic_motion.reset()
        if self.active_entry["uses_mimic"]:
            frame = self.mimic_motion.frame_index
            self.data.qpos[:3] = self.mimic_motion.body_pos_w[frame, 0]
            self.data.qpos[3:7] = self.mimic_motion.body_quat_w[frame, 0]
            self.data.qpos[self.qpos_indices] = self.mimic_motion.joint_pos[frame]
            self.data.qvel[:3] = self.mimic_motion.body_lin_vel_w[frame, 0]
            self.data.qvel[3:6] = self.mimic_motion.body_ang_vel_w[frame, 0]
            self.data.qvel[self.qvel_indices] = self.mimic_motion.joint_vel[frame]
            self.target_joint_pos[:] = self.mimic_motion.joint_pos[frame]
        else:
            root_height = (
                self.elastic_band.suspension_height
                if self.elastic_band.enabled
                else self.profile.root_height
            )
            self.data.qpos[:3] = (0.0, 0.0, root_height)
            self.data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
            self.data.qpos[self.qpos_indices] = self.profile.default_joint_pos
            self.target_joint_pos[:] = self.profile.default_joint_pos
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
            joint_pos - self.profile.default_joint_pos,
            joint_vel,
            self.previous_action,
        ]
        if self.active_entry["uses_left_arm"]:
            observation_parts.append(self.left_arm_trajectory.observation())
        observation = np.concatenate(observation_parts)
        return np.clip(observation, -100.0, 100.0).astype(np.float32)

    def _mimic_observation(self) -> np.ndarray:
        """Build Mimic input even while walk remains the active policy."""
        joint_pos = self.data.qpos[self.qpos_indices]
        joint_vel = self.data.qvel[self.qvel_indices]
        body_angular_velocity = self.data.sensor("BodyGyro").data.copy()
        robot_anchor_quat = self.data.xquat[self.model.body(MIMIC_ANCHOR_BODY_NAME).id]
        relative_anchor_quat = quaternion_multiply(
            quaternion_conjugate(robot_anchor_quat),
            self.mimic_motion.anchor_quaternion,
        )
        relative_anchor_rotation_6d = quaternion_to_rotation_matrix(
            relative_anchor_quat
        )[:, :2].reshape(-1)
        observation = np.concatenate(
            (
                self.mimic_motion.command,
                relative_anchor_rotation_6d,
                body_angular_velocity,
                joint_pos - self.profile.default_joint_pos,
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
            self.profile.default_joint_pos + self.ACTION_SCALE * requested_action,
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
            self._set_applied_target(next_target, self.mimic_policy_target_speed)
        else:
            self.target_joint_pos = next_target
            self.previous_action = np.clip(
                (self.target_joint_pos - self.profile.default_joint_pos) / self.ACTION_SCALE,
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
        self._apply_mimic_transition_support()
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
            if self.mimic_transition_state is not None:
                self._update_mimic_joint_transition()
            else:
                self.update_policy()
        for _ in range(self.CONTROL_DECIMATION):
            self.physics_step()
        if not self.damping_stopped:
            self._finish_mimic_joint_transition_if_ready()
        if self.active_entry["uses_left_arm"]:
            self.left_arm_trajectory.advance(self.SIM_DT * self.CONTROL_DECIMATION)
        # Hold the first reference frame while blending in from walk.  This
        # preserves the entire clip instead of consuming its first 0.5 s during
        # the policy hand-off.
        mimic_ready_to_advance = (
            self.mimic_transition_state is None
            and self.active_entry["uses_mimic"]
            and self.policy_transition_start is None
        )
        if mimic_ready_to_advance and self.mimic_motion.advance():
            walk_state = self.return_to_walk()
            if walk_state is None:
                print(
                    f"Mimic reference finished at frame {self.mimic_motion.frame_index}; "
                    "holding the final frame. Press R to replay."
                )
            else:
                print(
                    f"Mimic reference finished at frame {self.mimic_motion.frame_index}; "
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
        "stand+walk pair with X-button switching.",
    )
    parser.add_argument(
        "--stand-policy",
        type=Path,
        default=DEFAULT_STAND_POLICY,
        help="Stand policy for dual-policy mode (ignored when --policy is set).",
    )
    parser.add_argument(
        "--walk-policy",
        type=Path,
        default=DEFAULT_WALK_POLICY,
        help="Walk policy for dual-policy mode (ignored when --policy is set).",
    )
    parser.add_argument(
        "--mimic-policy",
        type=Path,
        default=None,
        help="Optional one-shot Mimic policy loaded alongside stand+walk. Press M "
        "from walk to play it; completion returns to walk automatically.",
    )
    parser.add_argument("--dof", type=int, choices=(12, 27), default=27)
    parser.add_argument(
        "--mode",
        choices=("locomotion", "stand", "mimic"),
        default="locomotion",
        help="Policy command semantics and keyboard bindings. In dual-policy "
        "mode this selects the initially active policy.",
    )
    parser.add_argument(
        "--motion-file",
        type=Path,
        default=None,
        help="Training motion NPZ for a direct or one-shot Mimic policy (must be 50 Hz).",
    )
    parser.add_argument(
        "--mimic-start-frame",
        type=int,
        default=0,
        help="Reference frame used to initialize and start Mimic playback.",
    )
    parser.add_argument(
        "--mimic-prepare-time",
        type=float,
        default=0.0,
        help=(
            "Seconds used for a fixed, rate-limited joint trajectory from walk "
            "to the first Mimic reference pose. Zero preserves the legacy hand-off."
        ),
    )
    parser.add_argument(
        "--mimic-recover-time",
        type=float,
        default=0.0,
        help=(
            "Seconds used for a fixed, rate-limited joint trajectory from the "
            "last Mimic target to the walk-ready default pose."
        ),
    )
    parser.add_argument(
        "--mimic-transition-timeout",
        type=float,
        default=2.0,
        help="Seconds after a transition trajectory before reporting a failed readiness gate.",
    )
    parser.add_argument(
        "--mimic-transition-target-speed",
        type=float,
        default=1.0,
        help="Prepare/recover joint-target speed limit in rad/s.",
    )
    parser.add_argument(
        "--mimic-policy-target-speed",
        type=float,
        default=6.0,
        help="Mimic-policy joint-target speed limit in rad/s.",
    )
    parser.add_argument(
        "--mimic-ready-rms",
        type=float,
        default=0.12,
        help="Maximum joint-position RMS error before a Mimic state hand-off [rad].",
    )
    parser.add_argument(
        "--mimic-ready-max-error",
        type=float,
        default=0.25,
        help="Maximum single-joint error before a Mimic state hand-off [rad].",
    )
    parser.add_argument(
        "--mimic-ready-max-velocity",
        type=float,
        default=0.8,
        help="Maximum absolute joint velocity before a Mimic state hand-off [rad/s].",
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
        default=None,
        help=(
            "Trajectory CSV for a stand-leftarm policy. Defaults to "
            "left_wrist_pitch_traj.csv next to this script."
        ),
    )
    parser.add_argument(
        "--left-arm-start-phase",
        type=float,
        default=1.5,
        help="Trajectory phase [s] the left-arm blend starts from. Phases in "
        "0.75-3.75 keep the blend path collision-free; 0 sweeps the wrist "
        "through the hip.",
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
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    if args.gamepad_index < 0:
        parser.error("--gamepad-index must be non-negative")
    if not 0.0 <= args.gamepad_deadzone < 1.0:
        parser.error("--gamepad-deadzone must be in [0, 1)")
    if args.gamepad and args.headless:
        parser.error("--gamepad cannot be combined with --headless")
    if args.policy is not None and args.mimic_policy is not None:
        parser.error("--policy and --mimic-policy cannot be used together")
    has_mimic_policy = args.mimic_policy is not None or (
        args.policy is not None and args.mode == "mimic"
    )
    if args.mode == "mimic" and not has_mimic_policy:
        parser.error("--mode mimic requires --policy or --mimic-policy")
    if has_mimic_policy and args.motion_file is None:
        parser.error("Mimic playback requires --motion-file")
    if not has_mimic_policy and args.motion_file is not None:
        parser.error("--motion-file requires --mode mimic or --mimic-policy")
    if args.mimic_start_frame < 0:
        parser.error("--mimic-start-frame must be non-negative")
    if min(
        args.mimic_prepare_time,
        args.mimic_recover_time,
        args.mimic_transition_timeout,
        args.mimic_ready_rms,
        args.mimic_ready_max_error,
        args.mimic_ready_max_velocity,
    ) < 0.0:
        parser.error("Mimic transition durations and tolerances must be non-negative")
    if args.mimic_transition_target_speed <= 0.0 or args.mimic_policy_target_speed <= 0.0:
        parser.error("Mimic target speed limits must be positive")
    return args


def main() -> None:
    args = parse_args()
    if args.policy is not None:
        policy_specs = [PolicySpec(args.mode, args.mode, args.policy.resolve())]
    else:
        policies_by_mode = {
            "stand": PolicySpec("stand", "stand", args.stand_policy.resolve()),
            "locomotion": PolicySpec("walk", "locomotion", args.walk_policy.resolve()),
        }
        if args.mimic_policy is not None:
            policies_by_mode["mimic"] = PolicySpec(
                "mimic", "mimic", args.mimic_policy.resolve()
            )
        mode_order = [args.mode] + [
            mode for mode in ("locomotion", "stand", "mimic") if mode != args.mode
        ]
        policy_specs = [policies_by_mode[mode] for mode in mode_order if mode in policies_by_mode]
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
        mimic_motion_path=args.motion_file,
        mimic_start_frame=args.mimic_start_frame,
        left_arm_traj_path=args.left_arm_traj,
        left_arm_enabled=args.left_arm_enabled,
        left_arm_start_phase=args.left_arm_start_phase,
        elastic_band_enabled=elastic_band_enabled,
        band_lift=args.band_lift,
        band_anchor_height=args.band_anchor_height,
        band_stiffness=args.band_stiffness,
        band_damping=args.band_damping,
        band_support_ratio=args.band_support_ratio,
        mimic_prepare_time=args.mimic_prepare_time,
        mimic_recover_time=args.mimic_recover_time,
        mimic_transition_timeout=args.mimic_transition_timeout,
        mimic_transition_target_speed=args.mimic_transition_target_speed,
        mimic_policy_target_speed=args.mimic_policy_target_speed,
        mimic_ready_rms=args.mimic_ready_rms,
        mimic_ready_max_error=args.mimic_ready_max_error,
        mimic_ready_max_velocity=args.mimic_ready_max_velocity,
    )
    simulator.stand(args.stand_seconds)
    gamepad = (
        GamepadCommandSource(args.gamepad_index, args.gamepad_deadzone)
        if args.gamepad
        else None
    )
    if simulator.left_arm_trajectory is not None:
        state = "ON" if simulator.left_arm_trajectory.enabled else "OFF"
        print(f"Left-arm trajectory: {state} (L toggles tracking).")
    loaded_names = "/".join(entry["name"] for entry in simulator.policy_entries)
    if initial_mode == "mimic":
        print(
            f"Loaded {args.dof}-DOF Mimic policy: {loaded_names}. Reference: "
            f"{simulator.mimic_motion.path} ({simulator.mimic_motion.frame_count} frames, "
            f"{simulator.mimic_motion.fps:g} Hz, start={simulator.mimic_motion.start_frame})."
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
    if simulator.mimic_motion is not None and initial_mode != "mimic":
        print(
            f"One-shot Mimic ready: {simulator.mimic_motion.path} "
            f"({simulator.mimic_motion.frame_count} frames, "
            f"{simulator.mimic_motion.fps:g} Hz). Press M from walk to start."
        )

    start_time = time.perf_counter()
    control_dt = simulator.SIM_DT * simulator.CONTROL_DECIMATION

    if args.headless:
        if args.duration > 0.0:
            duration = args.duration
        elif simulator.mimic_motion is not None:
            duration = simulator.mimic_motion.duration
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

    glfw_key_right = 262
    glfw_key_left = 263
    glfw_key_down = 264
    glfw_key_up = 265

    def print_status() -> None:
        band_status = "ON" if simulator.elastic_band.enabled else "OFF"
        arm_status = (
            "N/A"
            if simulator.left_arm_trajectory is None
            else "ON" if simulator.left_arm_trajectory.enabled else "OFF"
        )
        if simulator.mimic_transition_state is not None:
            transition_time = min(
                simulator.mimic_transition_elapsed,
                simulator.mimic_transition_duration,
            )
            command_status = (
                f"state={simulator.control_state}, "
                f"transition={transition_time:.2f}/{simulator.mimic_transition_duration:.2f}s"
            )
        elif simulator.active_mode == "mimic":
            command_status = (
                f"reference={simulator.mimic_motion.frame_index + 1}/"
                f"{simulator.mimic_motion.frame_count} "
                f"({simulator.mimic_motion.frame_index / simulator.mimic_motion.fps:.2f}s)"
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

    def start_mimic() -> None:
        if not simulator.has_mimic_policy:
            print("Mimic trigger ignored: no --mimic-policy is loaded.")
        elif simulator.damping_stopped:
            print("Cannot start Mimic during damping stop; press R to reset.")
        elif simulator.mimic_in_progress:
            print(f"Mimic trigger ignored: {simulator.control_state} is already active.")
        elif simulator.active_mode != "locomotion":
            print("Mimic can only start from walk mode; press P to switch to walk first.")
        else:
            print(f"Active policy switched to: {simulator.start_mimic()}")

    def key_callback(keycode: int) -> None:
        handled = True
        if keycode in (ord("R"), ord("r")):
            simulator.reset()
            simulator.stand(args.stand_seconds)
        elif keycode in (ord("X"), ord("x"), 32):
            simulator.command[:] = 0.0
        elif keycode in (ord("P"), ord("p")):
            switch_policy()
        elif keycode in (ord("M"), ord("m")):
            start_mimic()
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
    if simulator.has_mimic_policy:
        print("Mimic: press M from walk to play once; it returns to walk automatically.")
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
        if simulator.has_mimic_policy:
            print("Gamepad Mimic: LT+D-pad Down starts one playback from walk.")
        print("Gamepad safety: LT+B damping stop; LB+RB clears the command and disables control.")
    print_status()
    try:
        with mujoco.viewer.launch_passive(
            simulator.model, simulator.data, key_callback=key_callback
        ) as viewer:
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
                    if gamepad.mimic_requested:
                        start_mimic()
                        print_status()
                    if gamepad.a_toggle_requested:
                        toggle_left_arm()
                        print_status()
                simulator.control_step()
                viewer.sync()
                if args.duration > 0.0 and time.perf_counter() - start_time >= args.duration:
                    break
                sleep_time = control_dt - (time.perf_counter() - step_start)
                if sleep_time > 0.0:
                    time.sleep(sleep_time)
    finally:
        if gamepad is not None:
            gamepad.close()


if __name__ == "__main__":
    main()

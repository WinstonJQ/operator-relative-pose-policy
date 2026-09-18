from __future__ import annotations

import math
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import Any, Literal

import numpy as np
from forge_msgs import JointState, Pose

from .config import MultiGroupRelativePosePolicyConfig, RelativePosePolicyConfig
from .kinematics import ForgeKinematicsAdapter, ForwardKinematics, matrix_to_pose, pose_to_matrix
from .state_cache import JointStateCache

TranslationFrame = Literal["tcp", "base"]


@dataclass(frozen=True, slots=True)
class RelativePoseCommand:
    """Typed domain command, independent of the Forge Tool wire protocol."""

    group_name: str
    target_frame: str
    reference: str
    translation_frame: TranslationFrame
    translation_m: tuple[float, float, float]
    orientation_mode: str
    rotation: tuple[float, ...] | None
    max_state_age_ms: int


@dataclass(frozen=True, slots=True)
class RelativePoseResolution:
    source_pose: Pose
    target_pose: Pose
    reference_frame: str
    target_frame: str
    translation_frame: TranslationFrame
    rotation_frame: str
    state_age_ms: float
    snapshot_version: int
    snapshot_received_ns: int


@dataclass(frozen=True, slots=True)
class RelativePoseResolutionError(Exception):
    code: str
    message: str
    retryable: bool = False
    details: Mapping[str, Any] = field(default_factory=dict)

    def __str__(self) -> str:
        return self.message


def _invalid(message: str) -> RelativePoseResolutionError:
    return RelativePoseResolutionError("MOTION_INVALID_ARGUMENT", message)


def _axis_angle_matrix(value: tuple[float, ...] | None) -> np.ndarray:
    if value is None or len(value) != 3:
        raise _invalid("axis_angle_rad must contain exactly x, y, and z")
    rotation_vector = np.asarray(value, dtype=np.float64)
    angle = float(np.linalg.norm(rotation_vector))
    if angle <= 1e-15:
        return np.eye(3, dtype=np.float64)
    x, y, z = rotation_vector / angle
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=np.float64)
    return (
        np.eye(3, dtype=np.float64)
        + math.sin(angle) * skew
        + (1.0 - math.cos(angle)) * (skew @ skew)
    )


def _rotation_matrix(mode: str, value: tuple[float, ...] | None) -> np.ndarray:
    if mode in ("preserve", "keep"):
        if value is not None:
            raise _invalid("orientation delta must be null when orientation is preserved")
        return np.eye(3, dtype=np.float64)
    if mode == "apply_delta":
        return _axis_angle_matrix(value)
    if mode == "quaternion":
        if value is None or len(value) != 4:
            raise _invalid("rotation must contain qx, qy, qz, and qw")
        qx, qy, qz, qw = value
        try:
            return pose_to_matrix(
                Pose(x=0.0, y=0.0, qx=qx, qy=qy, qz=qz, qw=qw)
            )[:3, :3]
        except ValueError as exc:
            raise _invalid(str(exc)) from exc
    if mode == "rpy":
        if value is None or len(value) != 3:
            raise _invalid("rotation must contain roll, pitch, and yaw")
        roll, pitch, yaw = value
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return np.array(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ],
            dtype=np.float64,
        )
    raise _invalid(
        "orientation_mode must be preserve or apply_delta "
        "(legacy extensions: keep, quaternion, rpy)"
    )


class RelativePoseResolver:
    """Resolve a typed relative-pose command against the latest robot state."""

    def __init__(
        self,
        config: RelativePosePolicyConfig,
        *,
        kinematics: ForwardKinematics | None = None,
        state_cache: JointStateCache | None = None,
        clock_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self.config = config
        self.kinematics = kinematics or ForgeKinematicsAdapter(config)
        self.state_cache = state_cache or JointStateCache()
        self._clock_ns = clock_ns

    def update_joint_state(self, state: JointState, now_ns: int | None = None) -> None:
        self.state_cache.update(state, self._clock_ns() if now_ns is None else now_ns)

    def resolve(self, command: RelativePoseCommand) -> RelativePoseResolution:
        if command.group_name != self.config.group_name:
            raise RelativePoseResolutionError(
                "MOTION_INVALID_GROUP", f"unknown group {command.group_name!r}"
            )
        if command.reference != "current":
            raise _invalid("reference must be 'current'")
        if command.target_frame != self.config.tip_frame:
            raise RelativePoseResolutionError(
                "MOTION_INVALID_FRAME",
                f"target_frame must equal configured tip frame {self.config.tip_frame!r}",
            )
        if command.translation_frame not in ("tcp", "base"):
            raise _invalid("translation_frame must be tcp or base")
        if len(command.translation_m) != 3 or any(
            not math.isfinite(value) for value in command.translation_m
        ):
            raise _invalid("translation_m must contain three finite values")
        if command.rotation is not None and any(
            not math.isfinite(value) for value in command.rotation
        ):
            raise _invalid("orientation delta values must be finite")
        if command.max_state_age_ms <= 0:
            raise _invalid("max_state_age_ms must be positive")

        now_ns = self._clock_ns()
        try:
            snapshot = self.state_cache.snapshot(
                self.config.joint_names,
                now_ns,
                command.max_state_age_ms * 1_000_000,
            )
        except ValueError as exc:
            raise RelativePoseResolutionError(
                "MOTION_INCOMPLETE_STATE", str(exc), retryable=True
            ) from exc
        if snapshot is None:
            raise RelativePoseResolutionError(
                "MOTION_NO_FRESH_ROBOT_STATE",
                "fresh complete JointState feedback is required",
                retryable=True,
                details={"max_state_age_ms": command.max_state_age_ms},
            )

        source_pose = self.kinematics.forward(snapshot.positions)
        source = pose_to_matrix(source_pose)
        target = source.copy()
        translation = np.asarray(command.translation_m, dtype=np.float64)
        target[:3, 3] += (
            source[:3, :3] @ translation
            if command.translation_frame == "tcp"
            else translation
        )
        rotation = _rotation_matrix(command.orientation_mode, command.rotation)
        if command.orientation_mode not in ("preserve", "keep"):
            target[:3, :3] = source[:3, :3] @ rotation

        state_age_ms = (now_ns - snapshot.received_ns) / 1_000_000
        return RelativePoseResolution(
            source_pose=source_pose,
            target_pose=matrix_to_pose(target),
            reference_frame=self.config.base_frame,
            target_frame=self.config.tip_frame,
            translation_frame=command.translation_frame,
            rotation_frame="tcp",
            state_age_ms=state_age_ms,
            snapshot_version=snapshot.version,
            snapshot_received_ns=snapshot.received_ns,
        )


class MultiGroupRelativePoseResolver:
    """Resolve relative-pose commands across multiple kinematic groups.

    Each group owns its own ``RelativePoseResolver`` (and therefore its own FK adapter),
    but all groups share a single ``JointStateCache``. A single ``joint_state`` update
    refreshes the shared cache once; resolution dispatches strictly by ``group_name`` and
    never falls back to another group.
    """

    def __init__(
        self,
        config: MultiGroupRelativePosePolicyConfig,
        *,
        kinematics: Mapping[str, ForwardKinematics] | None = None,
        state_cache: JointStateCache | None = None,
        clock_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self.config = config
        shared_cache = state_cache or JointStateCache()
        self._state_cache = shared_cache
        self._clock_ns = clock_ns
        self._resolvers: dict[str, RelativePoseResolver] = {}
        for group_name, group_config in config.groups.items():
            self._resolvers[group_name] = RelativePoseResolver(
                group_config,
                kinematics=(kinematics or {}).get(group_name),
                state_cache=shared_cache,
                clock_ns=clock_ns,
            )

    def update_joint_state(self, state: JointState, now_ns: int | None = None) -> None:
        self._state_cache.update(state, self._clock_ns() if now_ns is None else now_ns)

    def resolve(self, command: RelativePoseCommand) -> RelativePoseResolution:
        resolver = self._resolvers.get(command.group_name)
        if resolver is None:
            raise RelativePoseResolutionError(
                "MOTION_INVALID_GROUP", f"unknown group {command.group_name!r}"
            )
        return resolver.resolve(command)


__all__ = [
    "MultiGroupRelativePoseResolver",
    "RelativePoseCommand",
    "RelativePoseResolution",
    "RelativePoseResolutionError",
    "RelativePoseResolver",
    "TranslationFrame",
]

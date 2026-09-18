from __future__ import annotations

import math
import time
from collections.abc import Callable, Mapping
from typing import Any, cast

from forge_msgs import JointState, Pose

from forge_tool import (
    TOOL_ENDPOINT_PROTOCOL,
    ToolContext,
    ToolEndpointDescriptor,
    ToolEndpointError,
    ToolError,
    ToolOperationDescriptor,
    ToolRequest,
    ToolResult,
)

from .config import RelativePosePolicyConfig
from .kinematics import ForwardKinematics
from .resolver import (
    MultiGroupRelativePoseResolver,
    RelativePoseCommand,
    RelativePoseResolutionError,
    RelativePoseResolver,
    TranslationFrame,
)
from .state_cache import JointStateCache

RELATIVE_ENDPOINT_ID = "motion.relative_pose"
RELATIVE_OPERATION = "resolve"
RELATIVE_DESCRIPTOR = ToolEndpointDescriptor(
    protocol_version=TOOL_ENDPOINT_PROTOCOL,
    endpoint_id=RELATIVE_ENDPOINT_ID,
    operations=(
        ToolOperationDescriptor(name=RELATIVE_OPERATION, semantics="query", max_concurrency=1),
    ),
)

_BASE_ARGUMENTS = {
    "group_name",
    "target_frame",
    "reference",
    "translation_frame",
    "translation_m",
    "orientation_mode",
    "max_state_age_ms",
}


def _error(
    code: str,
    message: str,
    *,
    retryable: bool = False,
    details: Mapping[str, Any] | None = None,
) -> ToolEndpointError:
    return ToolEndpointError(
        ToolError(code=code, message=message, retryable=retryable, details=details or {})
    )


def _pose_json(pose: Pose) -> dict[str, float]:
    return {
        "x": pose.x,
        "y": pose.y,
        "z": pose.z,
        "qx": pose.qx,
        "qy": pose.qy,
        "qz": pose.qz,
        "qw": pose.qw,
    }


def _finite_vector(value: Any, length: int, name: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise _error("MOTION_INVALID_ARGUMENT", f"{name} must be a {length}-element array")
    if any(isinstance(item, bool) or not isinstance(item, int | float) for item in value):
        raise _error("MOTION_INVALID_ARGUMENT", f"{name} values must be numbers")
    result = tuple(float(item) for item in value)
    if any(not math.isfinite(item) for item in result):
        raise _error("MOTION_INVALID_ARGUMENT", f"{name} values must be finite")
    return result


def _xyz(value: Any, name: str) -> tuple[float, float, float]:
    fields = {"x", "y", "z"}
    if not isinstance(value, Mapping) or set(value) != fields:
        raise _error("MOTION_INVALID_ARGUMENT", f"{name} must contain exactly x, y, and z")
    values = []
    for field in ("x", "y", "z"):
        item = value[field]
        if isinstance(item, bool) or not isinstance(item, int | float):
            raise _error("MOTION_INVALID_ARGUMENT", f"{name}.{field} must be a number")
        item = float(item)
        if not math.isfinite(item):
            raise _error("MOTION_INVALID_ARGUMENT", f"{name}.{field} must be finite")
        values.append(item)
    return (values[0], values[1], values[2])


class RelativePoseQueryEndpoint:
    """Forge Query SPI adapter over a protocol-independent resolver."""

    def __init__(
        self,
        resolver: RelativePoseResolver | MultiGroupRelativePoseResolver,
        *,
        epoch_ms: Callable[[], int] = lambda: time.time_ns() // 1_000_000,
    ) -> None:
        self.resolver = resolver
        self._epoch_ms = epoch_ms

    async def query(self, request: ToolRequest, context: ToolContext) -> ToolResult:
        if context.deadline_ms is not None and self._epoch_ms() >= context.deadline_ms:
            raise _error("FORGE_DEADLINE_EXCEEDED", "the Tool invocation deadline has expired")
        args = request.arguments
        missing_base = sorted(_BASE_ARGUMENTS - set(args))
        if missing_base:
            raise _error(
                "MOTION_INVALID_ARGUMENT",
                "resolve requires exactly the documented arguments",
                details={"missing": missing_base, "unexpected": []},
            )
        orientation_mode = args["orientation_mode"]
        if not isinstance(orientation_mode, str):
            raise _error("MOTION_INVALID_ARGUMENT", "orientation_mode must be a string")
        orientation_argument = (
            "axis_angle_rad" if orientation_mode in ("preserve", "apply_delta") else "rotation"
        )
        expected_arguments = _BASE_ARGUMENTS | {orientation_argument}
        missing = sorted(expected_arguments - set(args))
        unexpected = sorted(set(args) - expected_arguments)
        if missing or unexpected:
            raise _error(
                "MOTION_INVALID_ARGUMENT",
                "resolve requires exactly the documented arguments",
                details={"missing": missing, "unexpected": unexpected},
            )

        group_name = args["group_name"]
        target_frame = args["target_frame"]
        reference = args["reference"]
        if not isinstance(group_name, str):
            raise _error("MOTION_INVALID_ARGUMENT", "group_name must be a string")
        if not isinstance(target_frame, str):
            raise _error("MOTION_INVALID_ARGUMENT", "target_frame must be a string")
        if not isinstance(reference, str):
            raise _error("MOTION_INVALID_ARGUMENT", "reference must be a string")
        translation_frame = args["translation_frame"]
        if translation_frame not in ("tcp", "base"):
            raise _error("MOTION_INVALID_ARGUMENT", "translation_frame must be tcp or base")
        translation = _xyz(args["translation_m"], "translation_m")
        orientation_value = args[orientation_argument]
        if orientation_argument == "axis_angle_rad":
            rotation = (
                None
                if orientation_value is None
                else _xyz(orientation_value, "axis_angle_rad")
            )
        else:
            expected_length = 4 if orientation_mode == "quaternion" else 3
            rotation = (
                None
                if orientation_value is None
                else _finite_vector(orientation_value, expected_length, "rotation")
            )
        max_age = args["max_state_age_ms"]
        if isinstance(max_age, bool) or not isinstance(max_age, int) or max_age <= 0:
            raise _error(
                "MOTION_INVALID_ARGUMENT", "max_state_age_ms must be a positive integer"
            )

        try:
            resolution = self.resolver.resolve(
                RelativePoseCommand(
                    group_name=group_name,
                    target_frame=target_frame,
                    reference=reference,
                    translation_frame=cast(TranslationFrame, translation_frame),
                    translation_m=translation,
                    orientation_mode=orientation_mode,
                    rotation=rotation,
                    max_state_age_ms=max_age,
                )
            )
        except RelativePoseResolutionError as exc:
            raise _error(
                exc.code,
                exc.message,
                retryable=exc.retryable,
                details=exc.details,
            ) from exc
        return ToolResult(
            status="succeeded",
            outputs={
                "source_pose": _pose_json(resolution.source_pose),
                "target_pose": _pose_json(resolution.target_pose),
                "frames": {
                    "reference_frame": resolution.reference_frame,
                    "target_frame": resolution.target_frame,
                    "translation_frame": resolution.translation_frame,
                    "rotation_frame": resolution.rotation_frame,
                },
                "state_age_ms": resolution.state_age_ms,
                "source_snapshot": {
                    "version": resolution.snapshot_version,
                    "received_monotonic_ns": resolution.snapshot_received_ns,
                    "age_ms": resolution.state_age_ms,
                    "time_basis": "policy_receive_monotonic",
                },
            },
        )


class RelativePoseQuery(RelativePoseQueryEndpoint):
    """Backward-compatible constructor; new code should compose resolver and endpoint."""

    def __init__(
        self,
        config: RelativePosePolicyConfig,
        *,
        kinematics: ForwardKinematics | None = None,
        state_cache: JointStateCache | None = None,
        clock_ns: Callable[[], int] = time.monotonic_ns,
        epoch_ms: Callable[[], int] = lambda: time.time_ns() // 1_000_000,
    ) -> None:
        super().__init__(
            RelativePoseResolver(
                config,
                kinematics=kinematics,
                state_cache=state_cache,
                clock_ns=clock_ns,
            ),
            epoch_ms=epoch_ms,
        )

    def update_joint_state(self, state: JointState, now_ns: int | None = None) -> None:
        self.resolver.update_joint_state(state, now_ns)


__all__ = [
    "RELATIVE_DESCRIPTOR",
    "RELATIVE_ENDPOINT_ID",
    "RELATIVE_OPERATION",
    "RelativePoseQuery",
    "RelativePoseQueryEndpoint",
]

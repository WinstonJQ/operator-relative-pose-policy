from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True, slots=True)
class RelativePosePolicyConfig:
    group_name: str
    joint_names: tuple[str, ...]
    urdf_path: Path
    base_frame: str
    tip_frame: str

    def __post_init__(self) -> None:
        for field_name in ("group_name", "base_frame", "tip_frame"):
            value = getattr(self, field_name)
            if not isinstance(value, str) or not value:
                raise ValueError(f"{field_name} must be a non-empty string")
        if not isinstance(self.joint_names, tuple) or not self.joint_names:
            raise ValueError("joint_names must be a non-empty tuple")
        if any(not isinstance(name, str) or not name for name in self.joint_names):
            raise ValueError("joint_names must contain non-empty strings")
        if len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("joint_names must be unique")
        if not isinstance(self.urdf_path, Path):
            raise ValueError("urdf_path must be a Path")


@dataclass(frozen=True, slots=True)
class MultiGroupRelativePosePolicyConfig:
    """Immutable mapping of group name -> single-group config, used when ``groups`` is present."""

    groups: Mapping[str, RelativePosePolicyConfig]

    def __post_init__(self) -> None:
        if not isinstance(self.groups, Mapping) or not self.groups:
            raise ValueError("groups must be a non-empty mapping")
        for group_name, group_config in self.groups.items():
            if not isinstance(group_name, str) or not group_name:
                raise ValueError("group names must be non-empty strings")
            if not isinstance(group_config, RelativePosePolicyConfig):
                raise ValueError(f"group {group_name!r} must be a RelativePosePolicyConfig")
            if group_config.group_name != group_name:
                raise ValueError(
                    f"group {group_name!r} config group_name mismatch: "
                    f"{group_config.group_name!r}"
                )


_SINGLE_GROUP_FIELDS = {"group_name", "joint_names", "urdf_path", "base_frame", "tip_frame"}
_GROUP_FIELDS = {"joint_names", "urdf_path", "base_frame", "tip_frame"}


def load_config(
    path: str | Path,
) -> RelativePosePolicyConfig | MultiGroupRelativePosePolicyConfig:
    config_path = Path(path).expanduser().resolve()
    data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("relative pose policy config must be a YAML mapping")
    return config_from_mapping(data, base_path=config_path.parent)


def config_from_mapping(
    data: dict[str, Any], *, base_path: Path | None = None
) -> RelativePosePolicyConfig | MultiGroupRelativePosePolicyConfig:
    if isinstance(data, Mapping) and "groups" in data:
        return _multi_group_config_from_mapping(data, base_path=base_path)
    return _single_group_config_from_mapping(data, base_path=base_path)


def _single_group_config_from_mapping(
    data: dict[str, Any], *, base_path: Path | None = None
) -> RelativePosePolicyConfig:
    expected = {"group_name", "joint_names", "urdf_path", "base_frame", "tip_frame"}
    missing = sorted(expected - set(data))
    unknown = sorted(set(data) - expected)
    if missing or unknown:
        raise ValueError(f"config fields differ: missing={missing}, unknown={unknown}")

    joint_names = data["joint_names"]
    if not isinstance(joint_names, list) or any(not isinstance(item, str) for item in joint_names):
        raise ValueError("joint_names must be a list of strings")
    urdf_value = data["urdf_path"]
    if not isinstance(urdf_value, str) or not urdf_value:
        raise ValueError("urdf_path must be a non-empty string")
    urdf_path = Path(urdf_value).expanduser()
    if not urdf_path.is_absolute():
        urdf_path = (base_path or Path.cwd()) / urdf_path
    return RelativePosePolicyConfig(
        group_name=data["group_name"],
        joint_names=tuple(joint_names),
        urdf_path=urdf_path.resolve(),
        base_frame=data["base_frame"],
        tip_frame=data["tip_frame"],
    )


def _multi_group_config_from_mapping(
    data: dict[str, Any], *, base_path: Path | None = None
) -> MultiGroupRelativePosePolicyConfig:
    mixed = sorted(_SINGLE_GROUP_FIELDS & set(data))
    if mixed:
        raise ValueError(f"config mixes top-level fields with groups: {mixed}")
    groups = data["groups"]
    if not isinstance(groups, Mapping) or not groups:
        raise ValueError("groups must be a non-empty mapping")

    parsed: dict[str, RelativePosePolicyConfig] = {}
    seen_joints: dict[str, str] = {}
    for group_name, group_data in groups.items():
        if not isinstance(group_name, str) or not group_name:
            raise ValueError("group names must be non-empty strings")
        if not isinstance(group_data, Mapping):
            raise ValueError(f"group {group_name!r} must be a mapping")
        missing = sorted(_GROUP_FIELDS - set(group_data))
        unknown = sorted(set(group_data) - _GROUP_FIELDS)
        if missing or unknown:
            raise ValueError(
                f"group {group_name!r} fields differ: missing={missing}, unknown={unknown}"
            )
        joint_names = group_data["joint_names"]
        if not isinstance(joint_names, list) or any(
            not isinstance(item, str) for item in joint_names
        ):
            raise ValueError(f"group {group_name!r} joint_names must be a list of strings")
        urdf_value = group_data["urdf_path"]
        if not isinstance(urdf_value, str) or not urdf_value:
            raise ValueError(f"group {group_name!r} urdf_path must be a non-empty string")
        urdf_path = Path(urdf_value).expanduser()
        if not urdf_path.is_absolute():
            urdf_path = (base_path or Path.cwd()) / urdf_path
        group_config = RelativePosePolicyConfig(
            group_name=group_name,
            joint_names=tuple(joint_names),
            urdf_path=urdf_path.resolve(),
            base_frame=group_data["base_frame"],
            tip_frame=group_data["tip_frame"],
        )
        for joint_name in group_config.joint_names:
            previous = seen_joints.get(joint_name)
            if previous is not None:
                raise ValueError(
                    f"joint {joint_name!r} appears in multiple groups: "
                    f"{previous!r} and {group_name!r}"
                )
            seen_joints[joint_name] = group_name
        parsed[group_name] = group_config
    return MultiGroupRelativePosePolicyConfig(groups=parsed)


__all__ = [
    "MultiGroupRelativePosePolicyConfig",
    "RelativePosePolicyConfig",
    "config_from_mapping",
    "load_config",
]

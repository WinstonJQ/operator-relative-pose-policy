from __future__ import annotations

import asyncio
from pathlib import Path

import pytest
from forge_msgs import JointState, Pose

from forge_tool import ToolContext, ToolEndpointError, ToolExecutionKey, ToolRequest
from relative_pose_policy_node.config import (
    MultiGroupRelativePosePolicyConfig,
    RelativePosePolicyConfig,
    config_from_mapping,
    load_config,
)
from relative_pose_policy_node.query import RelativePoseQueryEndpoint
from relative_pose_policy_node.resolver import (
    MultiGroupRelativePoseResolver,
    RelativePoseCommand,
    RelativePoseResolutionError,
    RelativePoseResolver,
)

LEFT_JOINTS = ("l1", "l2", "l3", "l4", "l5")
RIGHT_JOINTS = ("r1", "r2", "r3", "r4", "r5")


def _left_config() -> RelativePosePolicyConfig:
    return RelativePosePolicyConfig(
        group_name="xlerobot_left_arm",
        joint_names=LEFT_JOINTS,
        urdf_path=Path("unused.urdf"),
        base_frame="base_link",
        tip_frame="left_arm_tcp",
    )


def _right_config() -> RelativePosePolicyConfig:
    return RelativePosePolicyConfig(
        group_name="xlerobot_right_arm",
        joint_names=RIGHT_JOINTS,
        urdf_path=Path("unused.urdf"),
        base_frame="base_link",
        tip_frame="right_arm_tcp",
    )


def _multi_config() -> MultiGroupRelativePosePolicyConfig:
    return MultiGroupRelativePosePolicyConfig(
        groups={"xlerobot_left_arm": _left_config(), "xlerobot_right_arm": _right_config()}
    )


class _OffsetKinematics:
    def __init__(self, x_offset: float) -> None:
        self.x_offset = x_offset

    def forward(self, positions: tuple[float, ...]) -> Pose:
        return Pose(x=positions[0] + self.x_offset, y=positions[1], z=positions[2])


def _kinematics() -> dict[str, _OffsetKinematics]:
    return {
        "xlerobot_left_arm": _OffsetKinematics(100.0),
        "xlerobot_right_arm": _OffsetKinematics(200.0),
    }


def _state() -> JointState:
    # A single 12-joint state carrying both arms plus grippers; each group selects only 5.
    return JointState(
        name=[
            "l1", "l2", "l3", "l4", "l5",
            "r1", "r2", "r3", "r4", "r5",
            "left_gripper", "right_gripper",
        ],
        position=[0.1, 0.2, 0.3, 0.4, 0.5, 1.1, 1.2, 1.3, 1.4, 1.5, 0.0, 0.0],
        velocity=[],
        effort=[],
    )


def _command(group_name: str, target_frame: str, **updates) -> RelativePoseCommand:
    values = {
        "group_name": group_name,
        "target_frame": target_frame,
        "reference": "current",
        "translation_frame": "base",
        "translation_m": (0.1, 0.0, 0.0),
        "orientation_mode": "preserve",
        "rotation": None,
        "max_state_age_ms": 1000,
    }
    values.update(updates)
    return RelativePoseCommand(**values)


# --- configuration ---


def _group(joint_names, **updates) -> dict:
    values = {
        "joint_names": joint_names,
        "urdf_path": "robot.urdf",
        "base_frame": "base_link",
        "tip_frame": "tcp",
    }
    values.update(updates)
    return values


def test_legacy_single_group_format_still_parses(tmp_path: Path) -> None:
    path = tmp_path / "policy.yaml"
    path.write_text(
        "group_name: arm\n"
        "joint_names: [j1, j2]\n"
        "urdf_path: robot.urdf\n"
        "base_frame: base\n"
        "tip_frame: tcp\n",
        encoding="utf-8",
    )

    config = load_config(path)

    assert isinstance(config, RelativePosePolicyConfig)
    assert config.group_name == "arm"
    assert config.joint_names == ("j1", "j2")


def test_multigroup_config_parses() -> None:
    config = config_from_mapping(
        {
            "groups": {
                "xlerobot_left_arm": _group(["l1", "l2"]),
                "xlerobot_right_arm": _group(["r1", "r2"]),
            }
        }
    )

    assert isinstance(config, MultiGroupRelativePosePolicyConfig)
    assert set(config.groups) == {"xlerobot_left_arm", "xlerobot_right_arm"}
    assert config.groups["xlerobot_left_arm"].joint_names == ("l1", "l2")
    assert config.groups["xlerobot_right_arm"].tip_frame == "tcp"


def test_multigroup_urdf_path_resolves_relative_to_config(tmp_path: Path) -> None:
    path = tmp_path / "policy.yaml"
    path.write_text(
        "groups:\n"
        "  left:\n"
        "    joint_names: [l1]\n"
        "    urdf_path: models/robot.urdf\n"
        "    base_frame: base\n"
        "    tip_frame: tcp\n",
        encoding="utf-8",
    )

    config = load_config(path)

    assert config.groups["left"].urdf_path == (tmp_path / "models" / "robot.urdf").resolve()


@pytest.mark.parametrize(
    "mapping",
    [
        {"groups": {}},
        {**_group(["j1"]), "groups": {"a": _group(["j2"])}},
        {"group_name": "arm", "groups": {"a": _group(["j1"])}},
        {"groups": {"a": _group(["j1", "j1"])}},
        {"groups": {"a": _group([])}},
        {"groups": {"a": {**_group(["j1"]), "extra": True}}},
        {"groups": {"a": {**_group(["j1"]), "group_name": "b"}}},
        {"groups": {"a": _group(["j1"]), "b": _group(["j1"])}},
    ],
)
def test_multigroup_config_rejects_invalid(mapping) -> None:
    with pytest.raises(ValueError):
        config_from_mapping(mapping)


# --- resolver ---


def test_multigroup_dispatch_selects_group_and_only_its_joints() -> None:
    resolver = MultiGroupRelativePoseResolver(
        _multi_config(), kinematics=_kinematics(), clock_ns=lambda: 50_000_000
    )
    resolver.update_joint_state(_state(), now_ns=10_000_000)

    left = resolver.resolve(_command("xlerobot_left_arm", "left_arm_tcp"))
    right = resolver.resolve(_command("xlerobot_right_arm", "right_arm_tcp"))

    assert left.source_pose.x == pytest.approx(100.1)
    assert left.target_pose.x == pytest.approx(100.2)
    assert left.target_frame == "left_arm_tcp"
    assert right.source_pose.x == pytest.approx(201.1)
    assert right.target_pose.x == pytest.approx(201.2)
    assert right.target_frame == "right_arm_tcp"


def test_multigroup_shares_single_state_update() -> None:
    resolver = MultiGroupRelativePoseResolver(
        _multi_config(), kinematics=_kinematics(), clock_ns=lambda: 50_000_000
    )
    resolver.update_joint_state(_state(), now_ns=0)

    left = resolver.resolve(_command("xlerobot_left_arm", "left_arm_tcp"))
    right = resolver.resolve(_command("xlerobot_right_arm", "right_arm_tcp"))

    assert left.snapshot_version == 1
    assert right.snapshot_version == 1


def test_multigroup_unknown_group_is_invalid_group() -> None:
    resolver = MultiGroupRelativePoseResolver(
        _multi_config(), kinematics=_kinematics(), clock_ns=lambda: 50_000_000
    )
    resolver.update_joint_state(_state(), now_ns=0)

    with pytest.raises(RelativePoseResolutionError) as captured:
        resolver.resolve(_command("other_arm", "other_tcp"))

    assert captured.value.code == "MOTION_INVALID_GROUP"
    assert captured.value.retryable is False


def test_multigroup_mismatched_tcp_is_invalid_frame() -> None:
    resolver = MultiGroupRelativePoseResolver(
        _multi_config(), kinematics=_kinematics(), clock_ns=lambda: 50_000_000
    )
    resolver.update_joint_state(_state(), now_ns=0)

    with pytest.raises(RelativePoseResolutionError) as captured:
        resolver.resolve(_command("xlerobot_left_arm", "right_arm_tcp"))

    assert captured.value.code == "MOTION_INVALID_FRAME"


# --- endpoint ---


def _context() -> ToolContext:
    return ToolContext(
        execution_key=ToolExecutionKey("invoke-1", "attempt-1"),
        tool_id="motion.relative",
        implementation_id="relative-policy",
        endpoint_id="motion.relative_pose",
        operation="resolve",
    )


def _arguments(group_name: str = "xlerobot_left_arm", target_frame: str = "left_arm_tcp"):
    return {
        "group_name": group_name,
        "target_frame": target_frame,
        "reference": "current",
        "translation_frame": "base",
        "translation_m": {"x": 0.1, "y": 0.0, "z": 0.0},
        "orientation_mode": "preserve",
        "axis_angle_rad": None,
        "max_state_age_ms": 1000,
    }


def test_multigroup_endpoint_preserves_contract() -> None:
    resolver = MultiGroupRelativePoseResolver(
        _multi_config(), kinematics=_kinematics(), clock_ns=lambda: 50_000_000
    )
    endpoint = RelativePoseQueryEndpoint(resolver)
    resolver.update_joint_state(_state(), now_ns=10_000_000)

    result = asyncio.run(endpoint.query(ToolRequest(_arguments()), _context()))

    assert result.status == "succeeded"
    assert result.outputs["source_pose"]["x"] == pytest.approx(100.1)
    assert result.outputs["target_pose"]["x"] == pytest.approx(100.2)
    assert result.outputs["frames"]["reference_frame"] == "base_link"
    assert result.outputs["frames"]["target_frame"] == "left_arm_tcp"
    assert "joint_state" not in result.outputs
    assert "positions" not in result.outputs


def test_multigroup_endpoint_unknown_group_error_code() -> None:
    resolver = MultiGroupRelativePoseResolver(
        _multi_config(), kinematics=_kinematics(), clock_ns=lambda: 50_000_000
    )
    endpoint = RelativePoseQueryEndpoint(resolver)
    resolver.update_joint_state(_state(), now_ns=0)

    with pytest.raises(ToolEndpointError) as captured:
        asyncio.run(endpoint.query(ToolRequest(_arguments("nope", "nope")), _context()))

    assert captured.value.error.code == "MOTION_INVALID_GROUP"


# --- real kinematics (ForgeKinematicsAdapter per group) ---

_TWO_ARM_URDF = """\
<?xml version="1.0"?>
<robot name="two_arm">
  <link name="base"/>
  <link name="left_link"/>
  <joint name="l1" type="revolute">
    <parent link="base"/>
    <child link="left_link"/>
    <origin xyz="-1 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
  <link name="left_arm_tcp"/>
  <joint name="left_tip" type="fixed">
    <parent link="left_link"/>
    <child link="left_arm_tcp"/>
    <origin xyz="1 0 0"/>
  </joint>
  <link name="right_link"/>
  <joint name="r1" type="revolute">
    <parent link="base"/>
    <child link="right_link"/>
    <origin xyz="1 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
  <link name="right_arm_tcp"/>
  <joint name="right_tip" type="fixed">
    <parent link="right_link"/>
    <child link="right_arm_tcp"/>
    <origin xyz="1 0 0"/>
  </joint>
</robot>
"""


def test_multigroup_real_kinematics_matches_independent_single_group(tmp_path: Path) -> None:
    urdf = tmp_path / "two_arm.urdf"
    urdf.write_text(_TWO_ARM_URDF, encoding="utf-8")
    left_cfg = RelativePosePolicyConfig(
        group_name="left", joint_names=("l1",), urdf_path=urdf, base_frame="base",
        tip_frame="left_arm_tcp",
    )
    right_cfg = RelativePosePolicyConfig(
        group_name="right", joint_names=("r1",), urdf_path=urdf, base_frame="base",
        tip_frame="right_arm_tcp",
    )
    multi = MultiGroupRelativePoseResolver(
        MultiGroupRelativePosePolicyConfig(groups={"left": left_cfg, "right": right_cfg}),
        clock_ns=lambda: 50_000_000,
    )
    single_left = RelativePoseResolver(left_cfg, clock_ns=lambda: 50_000_000)
    single_right = RelativePoseResolver(right_cfg, clock_ns=lambda: 50_000_000)

    state = JointState(
        name=["l1", "r1", "gripper"],
        position=[0.5, 0.25, 0.0],
        velocity=[],
        effort=[],
    )
    multi.update_joint_state(state, now_ns=0)
    single_left.update_joint_state(state, now_ns=0)
    single_right.update_joint_state(state, now_ns=0)

    ml = multi.resolve(_command("left", "left_arm_tcp"))
    mr = multi.resolve(_command("right", "right_arm_tcp"))
    sl = single_left.resolve(_command("left", "left_arm_tcp"))
    sr = single_right.resolve(_command("right", "right_arm_tcp"))

    for field in ("x", "y", "z", "qx", "qy", "qz", "qw"):
        assert getattr(ml.source_pose, field) == pytest.approx(getattr(sl.source_pose, field))
        assert getattr(mr.source_pose, field) == pytest.approx(getattr(sr.source_pose, field))
    # The two chains are physically distinct, so their TCP poses must differ.
    assert (ml.source_pose.x, ml.source_pose.y) != (mr.source_pose.x, mr.source_pose.y)
    assert ml.target_frame == "left_arm_tcp"
    assert mr.target_frame == "right_arm_tcp"

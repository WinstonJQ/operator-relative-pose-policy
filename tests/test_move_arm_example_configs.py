from __future__ import annotations

import os
from pathlib import Path

import pytest
import yaml


def _workspace_root() -> Path:
    configured = os.environ.get("FORGE_WORKSPACE_ROOT")
    candidates = [Path(configured).expanduser().resolve()] if configured else []
    candidates.extend(Path(__file__).resolve().parents)
    for candidate in candidates:
        skill = (
            candidate
            / "PhyAgentOS"
            / "examples"
            / "forge-skills"
            / "move-arm-by-ee"
        )
        if (skill / "profiles" / "mujoco" / "dataflow.yaml").is_file():
            return candidate
    pytest.skip(
        "PhyAgentOS Skill source not available; "
        "set FORGE_WORKSPACE_ROOT to run workspace contract tests",
        allow_module_level=True,
    )
    raise AssertionError("unreachable")


WORKSPACE_ROOT = _workspace_root()
SKILL_ROOT = (
    WORKSPACE_ROOT
    / "PhyAgentOS"
    / "examples"
    / "forge-skills"
    / "move-arm-by-ee"
)
PROFILE_ROOT = SKILL_ROOT / "profiles" / "mujoco"


def _load(name: str) -> dict:
    return yaml.safe_load((PROFILE_ROOT / name).read_text(encoding="utf-8"))


def _nodes() -> dict[str, dict]:
    return {node["id"]: node for node in _load("dataflow.yaml")["nodes"]}


def test_mujoco_profile_has_exact_nine_node_graph() -> None:
    assert set(_nodes()) == {
        "gateway",
        "relative_motion_policy",
        "motion_action_policy",
        "gripper_action_policy",
        "motion_server",
        "joint_trajectory_controller",
        "gripper_action_controller",
        "mujoco",
        "image_viewer",
    }


def test_mujoco_profile_closes_arm_and_gripper_loops() -> None:
    nodes = _nodes()
    assert nodes["mujoco"]["path"] == "${FORGE_RUNTIME_BIN}/mujoco_sim"
    assert nodes["mujoco"]["inputs"]["action/arm"] == (
        "joint_trajectory_controller/joint_command"
    )
    assert nodes["mujoco"]["inputs"]["action/gripper"] == (
        "gripper_action_controller/joint_command"
    )
    for consumer in (
        "relative_motion_policy",
        "motion_server",
        "joint_trajectory_controller",
        "gripper_action_controller",
    ):
        assert nodes[consumer]["inputs"]["joint_state"] == "mujoco/proprio_state"

    gateway = nodes["gateway"]
    policy = nodes["gripper_action_policy"]
    controller = nodes["gripper_action_controller"]
    assert gateway["inputs"]["gripper_tool_in"] == "gripper_action_policy/tool_out"
    assert policy["inputs"]["tool_in"] == "gateway/gripper_tool_out"
    assert policy["inputs"]["gripper_feedback"] == (
        "gripper_action_controller/gripper_feedback"
    )
    assert policy["inputs"]["gripper_result"] == (
        "gripper_action_controller/gripper_result"
    )
    assert controller["inputs"]["gripper_goal"] == (
        "gripper_action_policy/gripper_goal"
    )
    assert controller["inputs"]["gripper_cancel"] == (
        "gripper_action_policy/gripper_cancel"
    )


def test_tool_and_controller_configs_match_skill_contract() -> None:
    gateway = _load("gateway.yaml")
    specs = {spec["tool_id"]: spec for spec in gateway["tools"]["specs"]}
    assert set(specs) >= {
        "motion.resolve_relative_pose",
        "motion.move_pose",
        "gripper.set_opening",
    }
    gripper = specs["gripper.set_opening"]
    assert gripper["endpoint_id"] == "gripper.controller"
    assert gripper["semantics"] == "action"
    assert gripper["input_schema"]["properties"]["opening_m"] == {
        "type": "number",
        "minimum": 0,
        "maximum": 0.105,
    }

    controller = _load("gripper_controller.yaml")
    assert controller["joint_name"] == "gripper"
    assert controller["coordinate_type"] == "prismatic"
    assert controller["min_position"] == 0.0
    assert controller["max_position"] == 0.105

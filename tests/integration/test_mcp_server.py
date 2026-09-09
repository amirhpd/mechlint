"""The MCP tools, called the way a host calls them.

The chat layer is the headline outcome, so it is dogfooded from M1 while tool
names, descriptions and hints are still cheap to change. These tests are about
the contract a model sees -- names, read-only annotations, JSON shape, and what
a failure looks like -- not about the physics, which the other tests cover.

The tools are async, and they are driven here with ``asyncio.run`` rather than
an anyio plugin: mechlint runs its suite with plugin autoloading off, so that
a sourced ROS shell cannot inject its own pytest plugins into this venv.
"""

from __future__ import annotations

import asyncio
from pathlib import Path
from typing import Any

import pytest

mcp_server = pytest.importorskip("mechlint.mcp.server", reason="needs the optional [mcp] extra")

TOOLS = {"inspect_robot", "check_urdf", "compute_inertia"}


def call(name: str, **arguments: Any) -> dict[str, Any]:
    result = asyncio.run(mcp_server.server.call_tool(name, arguments))
    return dict(result.structured_content or {})


def listed() -> list[Any]:
    return asyncio.run(mcp_server.server.list_tools())


@pytest.fixture(scope="module")
def config(dast1_dir: Path) -> str:
    return str(dast1_dir / "mechlint.yaml")


def test_the_three_m1_tools_are_exposed() -> None:
    assert {tool.name for tool in listed()} == TOOLS


def test_every_tool_is_annotated_read_only() -> None:
    """llms.md promises the only writing tool is write_inertials, which is M3."""
    for tool in listed():
        assert tool.annotations is not None, tool.name
        assert tool.annotations.read_only_hint is True, tool.name


def test_every_tool_describes_itself() -> None:
    """A model picks a tool from its description; an empty one is a broken tool."""
    for tool in listed():
        assert tool.description and len(tool.description) > 80, tool.name


def test_the_server_tells_the_host_the_house_rules() -> None:
    """The 'numbers come from tools' policy has to reach the model, not just docs/llms.md."""
    assert "check_urdf" in mcp_server.INSTRUCTIONS
    assert "inspect_robot" in mcp_server.INSTRUCTIONS


def test_inspect_robot_returns_the_chain(config: str) -> None:
    payload = call("inspect_robot", config=config)

    assert payload["ok"] is True
    assert payload["robot"] == "dast_1"
    assert payload["reach_m"] == pytest.approx(0.759)
    assert payload["actuated_joints"] == [f"joint_{i}" for i in range(1, 7)]
    assert {link["name"] for link in payload["links"]} >= {"base_link", "tip"}


def test_check_urdf_returns_findings_with_ids_and_fixes(config: str) -> None:
    payload = call("check_urdf", config=config)
    findings = {f["check"]: f for f in payload["findings"]}

    assert payload["ok"] is False
    assert {"U001", "U005", "U006", "M001"} <= set(findings)
    assert findings["U001"]["fix"]
    assert "U007" in payload["skipped"]


def test_compute_inertia_says_where_each_mass_came_from(config: str) -> None:
    payload = call("compute_inertia", config=config)
    sources = {link["link"]: link["mass_source"] for link in payload["links"]}

    assert sources["base_link"] == "measured"
    assert sources["arm_2_link"] == "estimated"
    assert payload["total_mass_kg"] == pytest.approx(0.765, abs=0.001)


def test_a_missing_servo_mass_is_visible_in_the_json(config: str) -> None:
    """A servo silently absent is a torque number that will be wrong in M2."""
    payload = call("compute_inertia", config=config)
    pending = [c for link in payload["links"] for c in link["components"] if not c["ok"]]

    assert len(pending) == 5
    assert all("M2" in component["hint"] for component in pending)


def test_a_bad_path_is_a_result_not_a_traceback() -> None:
    """Soft errors with hints: the model gets something it can act on."""
    payload = call("check_urdf", description="does/not/exist.urdf")

    assert payload["ok"] is False
    assert "not found" in payload["error"]


def test_calling_with_nothing_says_what_is_missing() -> None:
    payload = call("inspect_robot")

    assert payload["ok"] is False
    assert "config" in payload["error"] and "description" in payload["error"]


def test_a_per_call_scale_overrides_the_config(dast1_dir: Path) -> None:
    """The what-if path M3 builds on: an argument beats the file, and nothing is edited."""
    payload = call(
        "inspect_robot",
        description=str(dast1_dir / "dast1.urdf"),
        model_scale="mm",
        package_paths={"description": str(dast1_dir)},
    )

    assert payload["model_scale"] == pytest.approx(0.001)
    assert payload["reach_m"] == pytest.approx(0.00759)

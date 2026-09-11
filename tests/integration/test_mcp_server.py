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

READ_ONLY = {"inspect_robot", "check_urdf", "compute_inertia", "torque_budget", "list_actuators"}
WRITES = {"write_inertials"}
TOOLS = READ_ONLY | WRITES


def call(name: str, **arguments: Any) -> dict[str, Any]:
    result = asyncio.run(mcp_server.server.call_tool(name, arguments))
    return dict(result.structured_content or {})


def listed() -> list[Any]:
    return asyncio.run(mcp_server.server.list_tools())


@pytest.fixture(scope="module")
def config(dast1_dir: Path) -> str:
    return str(dast1_dir / "mechlint.yaml")


def test_every_tool_built_so_far_is_exposed() -> None:
    assert {tool.name for tool in listed()} == TOOLS


def test_only_write_inertials_may_write() -> None:
    """llms.md promises exactly one writing tool. A host gates on this annotation,
    so getting it wrong on an inspect tool costs the user a permission prompt --
    and getting it wrong on the writing one costs them a file."""
    for tool in listed():
        assert tool.annotations is not None, tool.name
        assert tool.annotations.read_only_hint is (tool.name in READ_ONLY), tool.name


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
    assert {"U001", "U005", "U006", "U007", "M001"} <= set(findings)
    assert findings["U001"]["fix"]
    assert "D001" in payload["skipped"]  # silence about an unbuilt check is not a pass


def test_compute_inertia_says_where_each_mass_came_from(config: str) -> None:
    payload = call("compute_inertia", config=config)
    sources = {link["link"]: link["mass_source"] for link in payload["links"]}

    assert sources["base_link"] == "measured"
    assert sources["arm_2_link"] == "estimated"
    assert payload["total_mass_kg"] == pytest.approx(1.040, abs=0.001)


def test_every_servo_mass_resolves_and_says_where_it_came_from(config: str) -> None:
    """A servo silently absent is a torque number that is wrong in the safe-looking direction."""
    payload = call("compute_inertia", config=config)
    components = [c for link in payload["links"] for c in link["components"]]

    assert [c for c in components if not c["ok"]] == []
    servos = [c for c in components if c["label"] == "mg996r"]
    assert len(servos) == 5
    assert all("actuator db" in component["source"] for component in servos)


def test_torque_budget_judges_each_joint_against_its_servo(config: str) -> None:
    payload = call("torque_budget", config=config, payload_g=[0.0, 200.0])
    joints = {j["joint"]: j for j in payload["joints"]}

    assert payload["ok"] is False
    assert payload["gravity_source"] == "model"
    assert joints["joint_2"]["actuator"] == "mg996r"
    assert joints["joint_2"]["confidence"] == "low"
    assert joints["joint_2"]["cases"][-1]["ok"] is False
    assert joints["joint_6"]["cases"][-1]["ok"] is None  # no servo assigned: not judged
    assert payload["unjudged"] == ["joint_6"]


def test_torque_budget_echoes_the_overrides_it_was_given(config: str) -> None:
    """A what-if answer has to carry its own assumptions or it cannot be quoted."""
    payload = call("torque_budget", config=config, payload_g=[50.0], voltage=4.8, mount="ceiling")

    assert payload["payloads_g"] == [0.0, 50.0]  # zero is always included: it is the floor
    assert payload["voltage_V"] == 4.8
    assert payload["gravity_m_s2"] == pytest.approx([0.0, 0.0, 9.80665])
    assert payload["joints"][0]["voltage_V"] == 4.8


def test_list_actuators_answers_what_would_fit_instead(config: str) -> None:
    payload = call("list_actuators", min_torque_Nm=2.0, voltage=6.8)
    keys = [entry["key"] for entry in payload["actuators"]]

    assert keys == ["ds3225", "ds3218", "nema17_pg5"]  # the stepper counts, at 520 g
    assert all(entry["source_url"] for entry in payload["actuators"])
    assert payload["actuators"][0]["torque_at_voltage"]["basis"] == "stall torque at 6.8 V"


def test_list_actuators_returns_one_entry_with_its_source(config: str) -> None:
    payload = call("list_actuators", actuator="xl430_w250")

    assert payload["actuator"]["vendor"] == "ROBOTIS"
    assert payload["actuator"]["confidence"] == "high"
    assert "robotis.com" in payload["actuator"]["source_url"]


def test_list_actuators_also_reads_a_projects_own_table(tmp_path, config: str) -> None:
    """A project that ships its own servos should see them here, the way the CLI does."""
    (tmp_path / "my_servos.yaml").write_text(
        "actuators:\n  hs645:\n    name: HS-645MG\n    vendor: Hitec\n"
        "    kind: hobby_servo\n    stall_torque_Nm: { 6.0: 0.932 }\n"
        "    recommended_voltage: 6.0\n    mass_g: 55.2\n"
        "    source_url: http://example.invalid/hs645\n    source_date: 2026-09-10\n"
        "    source: invented for a test\n    confidence: low\n"
    )
    project = tmp_path / "mechlint.yaml"
    project.write_text(
        Path(config).read_text().replace("materials:", "actuator_db: my_servos.yaml\n\nmaterials:")
    )
    payload = call("list_actuators", config=str(project))

    assert "hs645" in {entry["key"] for entry in payload["actuators"]}
    assert "mg996r" in {entry["key"] for entry in payload["actuators"]}


@pytest.mark.parametrize(
    ("tool", "arguments", "expected"),
    [
        ("torque_budget", {"description": "does/not/exist.urdf"}, "not found"),
        ("torque_budget", {}, "give either"),
        ("list_actuators", {"actuator": "nope"}, "unknown actuator"),
        ("list_actuators", {"kind": "hobby-servo"}, "unknown kind"),
        ("list_actuators", {"actuator_db": "does/not/exist.yaml"}, "actuator table not found"),
    ],
)
def test_a_bad_argument_is_a_result_not_a_traceback(
    tool: str, arguments: dict[str, Any], expected: str
) -> None:
    """A tool that raises gives the model a stack trace it cannot act on. These give a
    sentence it can: what went wrong, in the same {ok, error, hint} shape as every other."""
    payload = call(tool, **arguments)

    assert payload["ok"] is False
    assert expected in payload["error"]


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


# --------------------------------------------------------------------------- overrides


def test_a_link_mass_override_answers_a_what_if_without_touching_the_file(config: str) -> None:
    """Suppose arm_1 came out at 80 g -- the whole point of the argument overrides."""
    before = call("torque_budget", config=config, payload_g=[200.0])
    after = call("torque_budget", config=config, payload_g=[200.0], link_mass_g={"arm_1_link": 80})
    worst = {j["joint"]: j["cases"][-1]["max_torque_Nm"] for j in before["joints"]}
    lighter = {j["joint"]: j["cases"][-1]["max_torque_Nm"] for j in after["joints"]}

    assert lighter["joint_2"] < worst["joint_2"]
    assert after["overrides"] == {"payload_g": [200.0], "link_mass_g": {"arm_1_link": 80.0}}
    assert before["overrides"] == {"payload_g": [200.0]}  # only what was actually overridden


def test_every_override_comes_back_in_the_result(config: str) -> None:
    """An answer that cannot say what it assumed cannot be quoted."""
    payload = call(
        "torque_budget",
        config=config,
        payload_g=[150.0],
        voltage=4.8,
        joint_actuators={"joint_2": "ds3225"},
        mount="ceiling",
        link_mass_g={"arm_2_link": 60.0},
    )

    assert payload["overrides"] == {
        "payload_g": [150.0],
        "voltage_V": 4.8,
        "joint_actuators": {"joint_2": "ds3225"},
        "mount": "ceiling",
        "link_mass_g": {"arm_2_link": 60.0},
    }


def test_an_unchanged_project_reports_no_overrides(config: str) -> None:
    assert call("compute_inertia", config=config)["overrides"] == {}


def test_a_mass_override_for_a_link_that_does_not_exist_is_refused(config: str) -> None:
    """Applying it to nothing would answer with the unmodified robot, which looks right."""
    payload = call("compute_inertia", config=config, link_mass_g={"arm_9_link": 80.0})

    assert payload["ok"] is False
    assert "no link called 'arm_9_link'" in payload["error"]


# --------------------------------------------------------------------------- writing


def test_write_inertials_writes_macros_and_says_what_it_skipped(tmp_path, config: str) -> None:
    target = tmp_path / "inertials.xacro"
    payload = call("write_inertials", config=config, output_path=str(target))

    assert payload["ok"] is True
    assert payload["created"] is True
    assert "base_link" in payload["links"]
    assert "mechlint_inertial_base_link" in target.read_text()
    # Not an error, but the reason has to be visible: a link missing from the file
    # silently is a link whose placeholder inertia stays in the robot.
    assert "wrist_link" in payload["skipped"]
    assert payload["include"] == '<xacro:include filename="inertials.xacro"/>'


def test_writing_twice_changes_nothing(tmp_path, config: str) -> None:
    """idempotent_hint=True is a promise to the host. This is the promise."""
    target = tmp_path / "inertials.xacro"
    call("write_inertials", config=config, output_path=str(target))
    first = target.read_text()
    again = call("write_inertials", config=config, output_path=str(target))

    assert again["unchanged"] is True
    assert again["created"] is False
    assert target.read_text() == first


def test_write_inertials_refuses_a_file_it_did_not_generate(tmp_path, config: str) -> None:
    target = tmp_path / "handwritten.xacro"
    target.write_text("<robot><!-- mine --></robot>")

    payload = call("write_inertials", config=config, output_path=str(target))

    assert payload["ok"] is False
    assert "not generated by mechlint" in payload["error"]
    assert "force" in payload["hint"]
    assert target.read_text() == "<robot><!-- mine --></robot>"


def test_a_directory_target_gets_the_default_filename(tmp_path, config: str) -> None:
    payload = call("write_inertials", config=config, output_path=str(tmp_path))

    assert Path(payload["path"]).name == "inertials.xacro"


# ------------------------------------------------------- the project's own actuator table


@pytest.fixture
def project_with_its_own_servo(tmp_path, config: str) -> str:
    """The DAST-1 config, plus a servo that exists only in this project's table."""
    (tmp_path / "my_servos.yaml").write_text(
        "actuators:\n  hs645:\n    name: HS-645MG\n    vendor: Hitec\n"
        "    kind: hobby_servo\n    stall_torque_Nm: { 6.0: 0.932 }\n"
        "    recommended_voltage: 6.0\n    mass_g: 55.2\n"
        "    source_url: http://example.invalid/hs645\n    source_date: 2026-09-10\n"
        "    source: invented for a test\n    confidence: low\n"
    )
    project = tmp_path / "mechlint.yaml"
    # Paths in a config are relative to the config, so a copy living somewhere else has
    # to point back at the fixture explicitly rather than at its own empty directory.
    fixture = Path(config).resolve().parent
    project.write_text(
        Path(config)
        .read_text()
        .replace("description: urdf/", f"description: {fixture}/urdf/")
        .replace(
            'description: ".", controller: "."',
            f'description: "{fixture}", controller: "{fixture}"',
        )
        .replace("materials:", "actuator_db: my_servos.yaml\n\nmaterials:")
        .replace("actuator: mg996r, drives: joint_1", "actuator: hs645, drives: joint_1")
        .replace("joint_1: mg996r", "joint_1: hs645")
    )
    return str(project)


def test_check_urdf_judges_u007_against_the_projects_own_table(
    project_with_its_own_servo: str,
) -> None:
    """U007 read only the bundled table, so a project's own servo came back as a FAIL that
    said the servo did not exist. `mechlint check` got this right and `check_urdf` did not:
    one project, two doors, two answers."""
    payload = call("check_urdf", config=project_with_its_own_servo)
    messages = " ".join(f["message"] for f in payload["findings"])

    assert "unknown actuator" not in messages


def test_compute_inertia_resolves_a_servo_only_the_project_knows(
    project_with_its_own_servo: str,
) -> None:
    """An unresolved servo is mass that quietly leaves the robot, which makes every
    torque number downstream wrong in the direction that looks safe."""
    payload = call("compute_inertia", config=project_with_its_own_servo)
    components = [c for link in payload["links"] for c in link["components"]]
    hs645 = [c for c in components if c["label"] == "hs645"]

    assert [c for c in components if not c["ok"]] == []
    assert hs645[0]["mass_kg"] == pytest.approx(0.0552)


def test_write_inertials_uses_the_projects_table_too(tmp_path, project_with_its_own_servo) -> None:
    """The file it writes carries the masses; the wrong database would bake in a lighter arm."""
    payload = call(
        "write_inertials",
        config=project_with_its_own_servo,
        output_path=str(tmp_path / "gen.xacro"),
    )

    assert payload["ok"] is True
    assert "base_link" in payload["links"]


def test_a_per_call_actuator_table_beats_the_config(tmp_path, config: str) -> None:
    """The CLI has --actuator-db; without this the chat could not try an unlisted servo
    at all, which is exactly the what-if the tool descriptions promise."""
    table = tmp_path / "one_servo.yaml"
    table.write_text(
        "actuators:\n  monster:\n    name: Monster\n    vendor: Nobody\n"
        "    kind: hobby_servo\n    stall_torque_Nm: { 6.0: 40.0 }\n"
        "    recommended_voltage: 6.0\n    mass_g: 500\n"
        "    source_url: http://example.invalid/monster\n    source_date: 2026-09-10\n"
        "    source: invented for a test\n    confidence: low\n"
    )
    payload = call(
        "torque_budget",
        config=config,
        actuator_db=str(table),
        joint_actuators={"joint_2": "monster"},
        payload_g=[200.0],
    )
    joints = {j["joint"]: j for j in payload["joints"]}

    assert joints["joint_2"]["actuator_name"] == "Monster"
    assert joints["joint_2"]["cases"][-1]["ok"] is True
    assert joints["joint_1"]["actuator"] == "mg996r"  # the bundled entries survive


def test_the_payload_can_be_hung_from_a_named_link(config: str) -> None:
    """The core has always taken this; neither door could reach it until now."""
    payload = call("torque_budget", config=config, payload_g=[200.0], payload_at="wrist_link")

    assert payload["payload_at"] == "wrist_link"
    assert payload["overrides"]["payload_at"] == "wrist_link"


def test_a_payload_link_that_does_not_exist_is_a_result_not_a_traceback(config: str) -> None:
    payload = call("torque_budget", config=config, payload_at="nose")

    assert payload["ok"] is False
    assert "no link called 'nose'" in payload["error"]

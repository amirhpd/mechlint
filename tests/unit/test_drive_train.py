"""The drive train as built: per-joint supply, gear stages, and which side a servo is on.

Every test here builds its own chain with ``urdf_builder``. The DAST-1 fixture is
frozen at an older design and is not the place to describe the new one.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest
import yaml
from pydantic import ValidationError
from typer.testing import CliRunner

from mechlint.cli.main import app
from mechlint.core import chain
from mechlint.core.actuators import bundled, load_database
from mechlint.core.checks import Severity, check_urdf
from mechlint.core.config import ActuatorsConfig, JointDrive, MechlintConfig
from mechlint.core.inertia import compute_inertia
from mechlint.core.report import render_torque, torque_markdown
from mechlint.core.torque import torque_budget
from mechlint.core.urdf import RobotModel
from tests.unit.urdf_builder import write_urdf

#: Two revolute joints with real effort limits, so U007 has something to judge, and an
#: origin on joint_2 that is both translated and rotated, so a servo placed in the wrong
#: frame lands somewhere visibly different. Masses come from `components` only.
ARM = """
  <link name="base"/>
  <link name="link_1"/>
  <link name="link_2"/>
  <link name="tip"/>
  <joint name="joint_1" type="revolute">
    <parent link="base"/><child link="link_1"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 1 0"/>
    <limit lower="-1.5708" upper="1.5708" effort="13" velocity="1"/>
  </joint>
  <joint name="joint_2" type="revolute">
    <parent link="link_1"/><child link="link_2"/>
    <origin xyz="0.4 0.02 0.03" rpy="0.3 0 0.5"/><axis xyz="0 1 0"/>
    <limit lower="-1.5708" upper="1.5708" effort="0.1" velocity="1"/>
  </joint>
  <joint name="joint_tip" type="fixed">
    <parent link="link_2"/><child link="tip"/><origin xyz="0.2 0 0"/>
  </joint>
"""

MASSES = """
components:
  link_1: [{ mass_g: 300, at: [0.2, 0, 0], note: arm }]
  link_2: [{ mass_g: 50,  at: [0.1, 0, 0], note: forearm }]
scenario: { mount: table, safety_factor: 1.0 }
"""

runner = CliRunner()


def _model(tmp_path: Path, body: str = ARM) -> RobotModel:
    return RobotModel.load(write_urdf(tmp_path, body))


def _config(tmp_path: Path, text: str) -> MechlintConfig:
    path = tmp_path / "mechlint.yaml"
    path.write_text("robot: { description: test.urdf }\n" + text)
    return MechlintConfig.from_yaml(path)


def _joints(report: object) -> dict[str, object]:
    return {joint.joint: joint for joint in report.joints}  # type: ignore[attr-defined]


# --------------------------------------------------------------------------- config


def test_short_long_and_null_forms_all_parse(tmp_path: Path) -> None:
    config = _config(
        tmp_path,
        "actuators:\n"
        "  voltage: 6.0\n"
        "  joints:\n"
        "    joint_1: { actuator: ds51150_12v, voltage: 12.0 }\n"
        "    joint_2: mg996r\n"
        "    joint_3: null\n"
        "    joint_4: { actuator: n20_50, transmission: { ratio: 2.0, efficiency: 0.95 } }\n",
    )
    drives = config.actuators.drives()

    assert drives["joint_1"] == JointDrive(actuator="ds51150_12v", voltage=12.0)
    assert drives["joint_2"] == JointDrive(actuator="mg996r", voltage=6.0)  # the default
    assert drives["joint_3"] is None
    assert drives["joint_4"].voltage == 6.0
    assert drives["joint_4"].transmission.factor == pytest.approx(1.9)


@pytest.mark.parametrize(
    "entry",
    [
        "{ actuator: mg996r, volts: 12 }",  # a typo in the long form
        "{ actuator: mg996r, transmission: { ratio: 2, eff: 0.9 } }",
        "{ actuator: mg996r, transmission: { ratio: 0 } }",
        "{ actuator: mg996r, transmission: { ratio: 2, efficiency: 1.2 } }",
        "{ voltage: 12 }",  # a voltage on its own drives nothing
    ],
)
def test_a_malformed_long_form_is_rejected(entry: str) -> None:
    with pytest.raises(ValidationError):
        ActuatorsConfig.model_validate({"joints": {"j1": yaml.safe_load(entry)}})


def test_with_no_voltage_anywhere_the_drive_has_none() -> None:
    """None, not a guess: the actuator's recommended voltage is then used, as before."""
    assert ActuatorsConfig(joints={"j1": "mg996r"}).drive("j1").voltage is None
    assert ActuatorsConfig(joints={"j1": "mg996r"}).drive("j2") is None


# --------------------------------------------------------------------------- per-joint supply


MIXED = MASSES + (
    "actuators:\n"
    "  voltage: 6.0\n"
    "  joints:\n"
    "    joint_1: { actuator: ds51150_12v, voltage: 12.0 }\n"
    "    joint_2: mg996r\n"
)


def test_each_joint_is_judged_at_its_own_supply(tmp_path: Path) -> None:
    report = torque_budget(_model(tmp_path), _config(tmp_path, MIXED), samples=64)
    joints = _joints(report)

    at_12 = bundled()["ds51150_12v"].torque_limit(12.0)
    at_6 = bundled()["mg996r"].torque_limit(6.0)

    assert joints["joint_1"].supply_V == 12.0
    assert joints["joint_1"].torque_basis == at_12.basis
    assert joints["joint_1"].cases[0].available_Nm == pytest.approx(at_12.torque_Nm)
    assert joints["joint_2"].supply_V == 6.0
    assert joints["joint_2"].torque_basis == at_6.basis
    # Two rails: no single number describes the report, and it says so.
    assert report.voltage_V is None
    assert "supply per joint" in render_torque(report)
    assert "supply per joint" in torque_markdown(report)


def test_u007_judges_each_joint_at_its_own_supply(tmp_path: Path) -> None:
    """joint_1 claims just under what the DS51150 gives at 12 V. Judged at the global
    6 V instead, it would be outside its 10-12.6 V range and U008 would fire. joint_2
    claims 0.1 N*m, well inside an MG996R at the default 6 V."""
    effort = 0.95 * bundled()["ds51150_12v"].torque_limit(12.0).torque_Nm
    body = ARM.replace('effort="13"', f'effort="{effort:.3f}"')
    report = check_urdf(_model(tmp_path, body), _config(tmp_path, MIXED))

    assert [f for f in report.findings if f.check in ("U007", "U008")] == []


def test_a_single_voltage_still_means_every_joint(tmp_path: Path) -> None:
    yaml = MASSES + "actuators: { voltage: 6.0, joints: { joint_1: mg996r, joint_2: mg996r } }"
    report = torque_budget(_model(tmp_path), _config(tmp_path, yaml), samples=64)

    assert report.voltage_V == 6.0
    assert {j.supply_V for j in report.joints} == {6.0}


def test_a_number_override_puts_every_joint_on_that_supply(tmp_path: Path) -> None:
    report = torque_budget(_model(tmp_path), _config(tmp_path, MIXED), voltage=12.6, samples=64)

    assert {j.supply_V for j in report.joints} == {12.6}
    assert report.voltage_V == 12.6
    assert report.overrides["voltage_V"] == 12.6


def test_a_dict_override_changes_only_the_joints_named(tmp_path: Path) -> None:
    report = torque_budget(
        _model(tmp_path), _config(tmp_path, MIXED), voltage={"joint_2": 4.8}, samples=64
    )
    joints = _joints(report)

    assert joints["joint_1"].supply_V == 12.0  # untouched: its own voltage in the config
    assert joints["joint_2"].supply_V == 4.8
    assert joints["joint_2"].torque_basis == "stall torque at 4.8 V"
    assert report.overrides["voltage_V"] == {"joint_2": 4.8}
    assert "joint_2 at 4.8 V" in render_torque(report)


def test_a_voltage_for_a_joint_that_does_not_exist_is_refused(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="no joint called 'joint_9'"):
        torque_budget(_model(tmp_path), _config(tmp_path, MIXED), voltage={"joint_9": 12.0})


def test_the_cli_takes_a_number_or_joint_pairs(tmp_path: Path) -> None:
    _model(tmp_path)
    config = tmp_path / "mechlint.yaml"
    config.write_text("robot: { description: test.urdf }\n" + MIXED)

    def run(*voltage: str) -> object:
        args = ["torque", "-c", str(config), "--samples", "64", "-f", "json"]
        for value in voltage:
            args += ["-V", value]
        return runner.invoke(app, args)

    assert json.loads(run("6.8").output)["overrides"]["voltage_V"] == 6.8
    pairs = json.loads(run("joint_1=12.6", "joint_2=4.8").output)
    assert pairs["overrides"]["voltage_V"] == {"joint_1": 12.6, "joint_2": 4.8}

    mixed = run("6", "joint_1=12")
    assert mixed.exit_code == 2
    assert "not both" in mixed.output


# --------------------------------------------------------------------------- transmission


def _geared(efficiency: str = "") -> str:
    stage = "{ ratio: 2.0" + (f", efficiency: {efficiency}" if efficiency else "") + " }"
    return MASSES + (
        "actuators:\n"
        "  joints:\n"
        f"    joint_2: {{ actuator: n20_50, voltage: 5.0, transmission: {stage} }}\n"
    )


def test_a_ratio_of_two_doubles_the_torque_at_the_joint(tmp_path: Path) -> None:
    joint = _joints(torque_budget(_model(tmp_path), _config(tmp_path, _geared()), samples=64))[
        "joint_2"
    ]

    assert joint.motor_torque_Nm == pytest.approx(0.059)
    assert joint.cases[0].available_Nm == pytest.approx(0.118)
    assert joint.torque_basis == (
        "stall torque at 5 V (0.059 N*m) x 2:1 stage, ideal (no efficiency given)"
    )


def test_efficiency_takes_its_share(tmp_path: Path) -> None:
    joint = _joints(torque_budget(_model(tmp_path), _config(tmp_path, _geared("0.9")), samples=64))[
        "joint_2"
    ]

    assert joint.cases[0].available_Nm == pytest.approx(0.059 * 1.8)
    assert joint.torque_basis == "stall torque at 5 V (0.059 N*m) x 2:1 stage x 0.9 efficiency"


def test_the_replacement_search_is_handed_the_motor_side_torque(tmp_path: Path) -> None:
    """Searching the database with the joint-side requirement would ask for a motor
    1.8x too strong: what the motor must give is the requirement divided by the stage."""
    joint = _joints(torque_budget(_model(tmp_path), _config(tmp_path, _geared("0.9")), samples=64))[
        "joint_2"
    ]
    case = joint.cases[0]

    assert case.motor_required_Nm == pytest.approx(case.required_Nm / 1.8)


def test_the_rendered_report_names_the_stage(tmp_path: Path) -> None:
    report = torque_budget(_model(tmp_path), _config(tmp_path, _geared("0.9")), samples=64)

    assert "2:1 stage x 0.9 efficiency" in render_torque(report)
    assert "2:1 stage x 0.9 efficiency" in torque_markdown(report)


def test_a_swapped_actuator_keeps_the_joints_gear_stage(tmp_path: Path) -> None:
    joint = _joints(
        torque_budget(
            _model(tmp_path),
            _config(tmp_path, _geared()),
            assignments={"joint_2": "mg996r"},
            samples=64,
        )
    )["joint_2"]

    assert joint.actuator == "mg996r"
    assert joint.transmission.ratio == 2.0
    assert joint.cases[0].available_Nm == pytest.approx(
        2 * bundled()["mg996r"].stall_torque_Nm[4.8]
    )


def test_u007_passes_an_effort_the_gear_stage_makes_reachable(tmp_path: Path) -> None:
    """joint_2 claims 0.1 N*m: more than the N20's 0.059, less than 0.118 through 2:1."""
    direct = MASSES + "actuators: { joints: { joint_2: { actuator: n20_50, voltage: 5.0 } } }"

    geared = check_urdf(_model(tmp_path), _config(tmp_path, _geared()))
    assert "U007" not in {f.check for f in geared.findings}

    plain = check_urdf(_model(tmp_path), _config(tmp_path, direct))
    (u007,) = [f for f in plain.findings if f.check == "U007"]
    assert "0.059 N*m at the joint" in u007.message


def test_the_transmission_matches_the_hand_made_workaround_entry(tmp_path: Path) -> None:
    """DAST-1 described joint_4 as a fake `n20_50_x2` entry: the N20's figures doubled
    by hand. The transmission must give the same joint torque without one."""
    workaround = tmp_path / "workaround.yaml"
    workaround.write_text(
        "actuators:\n"
        "  n20_50_x2:\n"
        "    name: N20 50:1 + 1:2 spur stage\n"
        "    vendor: generic\n"
        "    kind: dc_gearmotor\n"
        "    stall_torque_Nm: { 5.0: 0.118, 12.0: 0.282 }\n"
        "    recommended_voltage: 5.0\n"
        "    mass_g: 10.0\n"
        "    source_url: https://www.servocity.com/460-rpm-micro-gear-motor/\n"
        "    source_date: 2026-11-15\n"
        "    source: n20_50 doubled by hand\n"
    )
    database = load_database(workaround)

    def project(key: str, stage: str) -> MechlintConfig:
        return _config(
            tmp_path,
            MASSES.replace(
                "components:\n", f"components:\n  base: [{{ actuator: {key}, drives: joint_1 }}]\n"
            )
            + f"actuators: {{ joints: {{ joint_2: {{ actuator: {key}, voltage: 5.0{stage} }} }} }}",
        )

    model = _model(tmp_path)
    old = project("n20_50_x2", "")
    new = project("n20_50", ", transmission: { ratio: 2.0 }")
    before = torque_budget(model, old, actuators=database, samples=64)
    after = torque_budget(model, new, actuators=database, samples=64)

    assert _joints(after)["joint_2"].cases[0].available_Nm == pytest.approx(
        _joints(before)["joint_2"].cases[0].available_Nm
    )
    for name in ("joint_1", "joint_2"):
        assert _joints(after)[name].cases[0].max_torque_Nm == pytest.approx(
            _joints(before)[name].cases[0].max_torque_Nm
        )
    masses = [
        {link.link: link.mass_kg for link in compute_inertia(model, c, actuators=database).links}
        for c in (old, new)
    ]
    assert masses[0] == pytest.approx(masses[1])


# --------------------------------------------------------------------------- mounting side


def _servo_world_position(tmp_path: Path, listed_under: str, q: list[float]) -> np.ndarray:
    model = _model(tmp_path)
    config = _config(
        tmp_path, f"components: {{ {listed_under}: [{{ actuator: mg996r, drives: joint_2 }}] }}"
    )
    link = next(x for x in compute_inertia(model, config).links if x.link == listed_under)
    (servo,) = link.components
    assert servo.ok, servo.error
    pose = chain.forward_kinematics(model.urdf, np.array([q]))[listed_under][0]
    return (pose @ np.array([*servo.at_m, 1.0]))[:3]


@pytest.mark.parametrize("q", [[0.0, 0.0], [0.7, -1.1]])
def test_a_servo_lands_in_the_same_place_from_either_side_of_its_joint(
    tmp_path: Path, q: list[float]
) -> None:
    """Bolted to the parent or to the child, a servo is on its joint. Checked away from
    the zero pose too, where a frame mix-up cannot hide behind identity rotations."""
    parent = _servo_world_position(tmp_path, "link_1", q)
    child = _servo_world_position(tmp_path, "link_2", q)

    assert child == pytest.approx(parent, abs=1e-12)


def test_a_servo_on_an_unrelated_link_is_reported_not_guessed(tmp_path: Path) -> None:
    yaml = MASSES.replace(
        "link_2: [{ mass_g: 50",
        "base: [{ actuator: mg996r, drives: joint_2 }]\n  link_2: [{ mass_g: 50",
    )
    model, config = _model(tmp_path), _config(tmp_path, yaml)

    (d004,) = [f for f in check_urdf(model, config).findings if f.check == "D004"]
    assert d004.severity is Severity.FAIL
    assert "'link_1' and 'link_2'" in d004.message

    inertia = compute_inertia(model, config)
    (pending,) = inertia.pending_components
    assert "D004" in pending.error
    assert next(x for x in inertia.links if x.link == "base").mass_kg == 0.0

    assert "D004" in {f.check for f in torque_budget(model, config, samples=16).findings}


def test_at_still_wins_on_an_unrelated_link(tmp_path: Path) -> None:
    config = _config(
        tmp_path, "components: { base: [{ actuator: mg996r, drives: joint_2, at: [0, 0, 0.05] }] }"
    )

    assert "D004" not in {f.check for f in check_urdf(_model(tmp_path), config).findings}


# --------------------------------------------------------------------------- voltage fallback


def test_u007_says_when_the_supply_fell_back_and_u008_flags_the_rail(tmp_path: Path) -> None:
    """The DAST-1 case: a 10-12.6 V servo on a 6 V rail. U007 used to quote the 10 V
    figure as if it applied; now it says it over-states, and U008 fires on its own."""
    yaml = "actuators: { voltage: 6.0, joints: { joint_1: ds51150_12v } }"
    report = check_urdf(
        _model(tmp_path, ARM.replace('effort="13"', 'effort="20"')), _config(tmp_path, yaml)
    )

    (u007,) = [f for f in report.findings if f.check == "U007"]
    assert "OVER-states" in u007.message
    assert "below the 10-12.6 V operating range" in u007.message

    (u008,) = [f for f in report.findings if f.check == "U008"]
    assert u008.severity is Severity.WARN
    assert u008.subject == "joint_1"


def test_u008_fires_even_when_the_effort_limit_is_fine(tmp_path: Path) -> None:
    yaml = "actuators: { voltage: 6.0, joints: { joint_1: ds51150_12v } }"
    report = check_urdf(
        _model(tmp_path, ARM.replace('effort="13"', 'effort="1"')), _config(tmp_path, yaml)
    )

    assert "U007" not in {f.check for f in report.findings}
    assert "U008" in {f.check for f in report.findings}

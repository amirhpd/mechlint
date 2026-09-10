"""M2 on the real robot: does an MG996R hold DAST-1's joints?

The plan makes this milestone done when, on this fixture, mechlint prints a
per-joint worst-case torque at 0 / 100 / 200 g of payload and a pass or fail
against an MG996R at 6 V. These tests are that sentence, executable -- and the
answer, for the record, is no: joints 2, 3 and 4 are over budget, joint 2 before
anything is even picked up.

The numbers are pinned. They move when the meshes, the infill or the actuator
database move, and each of those is a deliberate change that should show up here
as a deliberate edit -- not as a loosened tolerance.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from typer.testing import CliRunner

from mechlint.cli.main import app
from mechlint.core.checks import Severity, check_urdf
from mechlint.core.config import MechlintConfig
from mechlint.core.torque import torque_budget
from mechlint.core.urdf import PackageResolver, RobotModel

runner = CliRunner()

PAYLOADS_G = [0.0, 100.0, 200.0]

#: Worst-case |torque| in N*m at 0 / 100 / 200 g, over the sampled joint space.
#: joint_1 turns about the mounting axis, so gravity applies no torque about it at
#: any pose -- that is a zero, not a missing number.
TORQUE_BASELINE_NM = {
    "joint_1": (0.0000, 0.0000, 0.0000),
    "joint_2": (1.0532, 1.6509, 2.2485),
    "joint_3": (0.5186, 0.9648, 1.4110),
    "joint_4": (0.1119, 0.3692, 0.6265),
    "joint_5": (0.0109, 0.1805, 0.3500),
    "joint_6": (0.0110, 0.1826, 0.3542),
}

#: MG996R at 6 V, from the bundled database: 11 kgf*cm.
MG996R_STALL_NM = 1.079


@pytest.fixture(scope="module")
def config(dast1_dir: Path) -> MechlintConfig:
    return MechlintConfig.from_yaml(dast1_dir / "mechlint.yaml")


@pytest.fixture(scope="module")
def model(config: MechlintConfig) -> RobotModel:
    resolver = PackageResolver(
        {name: config.resolve(path) for name, path in config.robot.package_paths.items()}
    )
    return RobotModel.load(
        config.description_path,
        xacro_args=config.robot.xacro_args,
        resolver=resolver,
        model_scale=float(config.robot.model_scale),
    )


@pytest.fixture(scope="module")
def budget(model: RobotModel, config: MechlintConfig):
    return torque_budget(model, config, payloads_g=PAYLOADS_G)


# --------------------------------------------------------------------------- the numbers


@pytest.mark.parametrize("joint", sorted(TORQUE_BASELINE_NM))
def test_the_worst_case_torques_hold(budget, joint: str) -> None:
    computed = next(entry for entry in budget.joints if entry.joint == joint)

    assert [case.max_torque_Nm for case in computed.cases] == pytest.approx(
        TORQUE_BASELINE_NM[joint], abs=1e-3
    )


def test_gravity_comes_from_the_model_not_the_config(budget) -> None:
    """The fixture has a world frame, so the world-to-base rotation decides which way is
    down -- and it is what the simulator runs. `scenario.mount` is not consulted."""
    assert budget.gravity_source == "model"
    assert budget.implied_mount == "table"
    assert budget.gravity_m_s2 == pytest.approx((0.0, 0.0, -9.80665))


def test_the_payload_hangs_from_the_tip_link(budget) -> None:
    assert budget.payload_at == "tip"


def test_a_joint_carries_everything_below_it_and_nothing_beside_it(budget) -> None:
    """The Kinect is bolted to base_link on a stick. It is downstream of nothing."""
    joint_2 = next(entry for entry in budget.joints if entry.joint == "joint_2")

    assert "kinect_link" not in joint_2.downstream_links
    assert joint_2.downstream_mass_kg == pytest.approx(0.4163, abs=1e-4)


# --------------------------------------------------------------------------- the verdict


def test_the_answer_is_no_for_joints_2_3_and_4(budget) -> None:
    """This is the motor decision for the 6-DOF upgrade, in one assertion."""
    failed = {entry.joint for entry in budget.joints if entry.ok is False}

    assert failed == {"joint_2", "joint_3", "joint_4"}
    assert not budget.ok


def test_joint_2_is_over_budget_before_it_picks_anything_up(budget) -> None:
    """1.05 N*m of arm against 1.08 N*m of stall torque: no margin at all, empty."""
    joint_2 = next(entry for entry in budget.joints if entry.joint == "joint_2")
    empty = joint_2.cases[0]

    assert empty.payload_kg == 0.0
    assert empty.max_torque_Nm > MG996R_STALL_NM / 2.0  # the safety factor is 2.0
    assert empty.ok is False


def test_t001_names_the_lightest_payload_that_fails(budget) -> None:
    """A joint that cannot hold the arm up empty is a different problem from one that
    runs out at 200 g, and the finding should say which without comparing columns."""
    findings = {f.subject: f for f in budget.findings if f.check == "T001"}

    assert set(findings) == {"joint_2", "joint_3", "joint_4"}
    assert "no payload" in findings["joint_2"].message
    assert "100 g payload" in findings["joint_3"].message
    assert "200 g payload" in findings["joint_4"].message
    assert all(f.severity is Severity.FAIL for f in findings.values())


def test_every_judged_joint_cites_a_sourced_datasheet(budget) -> None:
    """A margin is only worth what the number it is measured against is worth."""
    judged = [entry for entry in budget.joints if entry.judged]

    assert len(judged) == 5
    for entry in judged:
        assert entry.actuator_name == "MG996R"
        assert entry.torque_basis == "stall torque at 6 V"
        assert entry.confidence == "low"
        assert entry.source_url.startswith("http")


def test_joint_6_is_reported_but_not_judged(budget) -> None:
    """`joint_6: null` in mechlint.yaml means "no servo yet". Silence would read as a pass."""
    joint_6 = next(entry for entry in budget.joints if entry.joint == "joint_6")

    assert budget.unjudged == ["joint_6"]
    assert joint_6.cases[-1].max_torque_Nm == pytest.approx(0.3542, abs=1e-3)
    assert all(case.ok is None for case in joint_6.cases)


def test_the_shoulder_yaw_has_an_unbounded_margin_not_a_zero_one(budget) -> None:
    joint_1 = next(entry for entry in budget.joints if entry.joint == "joint_1")

    assert joint_1.cases[0].margin is None
    assert joint_1.ok is True
    assert "parallel to gravity" in joint_1.notes[0]


# --------------------------------------------------------------------------- U007


def test_u007_catches_the_twenty_newton_metre_effort_limits(
    model: RobotModel, config: MechlintConfig
) -> None:
    """Every DAST-1 joint declares effort="20.0" against a servo that gives 1.079 N*m.
    A controller clamping at twenty times what the motor can do is not clamping."""
    findings = [f for f in check_urdf(model, config).findings if f.check == "U007"]

    assert {f.subject for f in findings} == {f"joint_{i}" for i in range(1, 6)}
    assert all("18.5x" in f.message for f in findings)
    assert all(f.severity is Severity.FAIL for f in findings)


# --------------------------------------------------------------------------- what would fit


def test_the_database_can_say_what_would_hold_joint_2(budget) -> None:
    """The follow-up question, and the reason a curated database earns its keep."""
    from mechlint.core.actuators import bundled

    joint_2 = next(entry for entry in budget.joints if entry.joint == "joint_2")
    needed = joint_2.cases[-1].required_Nm
    found = bundled().matching(min_torque_Nm=needed, voltage=6.8, max_mass_g=100)

    assert needed == pytest.approx(2.0 * 2.2485, abs=1e-2)
    assert found == []  # nothing under 100 g holds 4.5 N*m: this arm needs a redesign, not a servo


# --------------------------------------------------------------------------- the CLI


def test_the_cli_prints_the_three_payload_columns(dast1_dir: Path) -> None:
    """The milestone's done-when, at the terminal."""
    result = runner.invoke(
        app,
        ["torque", "-c", str(dast1_dir / "mechlint.yaml"), "-P", "0", "-P", "100", "-P", "200"],
    )

    assert result.exit_code == 1
    for label in ("no payload", "100 g", "200 g"):
        assert label in result.output
    assert "worst margin" in result.output
    assert "safety factor 2" in result.output
    assert "joint_6" in result.output


def test_the_cli_can_try_a_different_servo_without_editing_the_file(dast1_dir: Path) -> None:
    result = runner.invoke(
        app,
        [
            "torque",
            "-c",
            str(dast1_dir / "mechlint.yaml"),
            "--actuator",
            "joint_2=ds3225",
            "--voltage",
            "6.8",
            "-f",
            "json",
        ],
    )
    payload = json.loads(result.output)
    joint_2 = next(j for j in payload["joints"] if j["joint"] == "joint_2")

    assert joint_2["actuator"] == "ds3225"
    assert joint_2["cases"][0]["available_Nm"] == pytest.approx(2.403)
    assert joint_2["cases"][0]["ok"] is True  # the shoulder alone is fine on a DS3225


def test_check_runs_both_halves_under_one_exit_code(dast1_dir: Path) -> None:
    result = runner.invoke(app, ["check", "-c", str(dast1_dir / "mechlint.yaml")])

    assert result.exit_code == 1
    assert "U001" in result.output  # the checks half
    assert "worst margin" in result.output  # the torque half

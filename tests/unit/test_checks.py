"""Every check, on a model built to fail exactly one of them.

The DAST-1 fixture proves the checks fire on a real robot; these prove they
fire for the reason they claim, and stay quiet otherwise.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from pydantic import ValidationError

from mechlint.core.checks import CHECKS, Severity, check_urdf
from mechlint.core.config import MechlintConfig
from mechlint.core.urdf import RobotModel
from tests.unit.urdf_builder import box, inertial, write_urdf

DOCS = Path(__file__).parents[2] / "docs" / "checks.md"

#: A metre-scale two-link arm whose inertials are consistent with its boxes, and
#: whose 1 m reach is plausible at scale 1 and at scale 0.1, but not at 0.001.
#: 0.5 kg box of 0.1 x 0.1 x 0.4: m/12 * (0.01 + 0.16) = 0.00708, and
#: m/12 * (0.01 + 0.01) = 0.000833 about its long axis.
SANE = f"""
  <link name="base">{inertial(0.5, (0.00708, 0.00708, 0.000833))}{box("0.1 0.1 0.4")}</link>
  <link name="tip">{inertial(0.2, (0.0031, 0.0031, 0.00033))}{box("0.1 0.1 0.4")}</link>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="tip"/>
    <origin xyz="0 0 1.0"/><axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="5" velocity="1"/>
  </joint>
"""


def _check(tmp_path: Path, body: str, *, model_scale: float = 1.0):
    model = RobotModel.load(write_urdf(tmp_path, body), model_scale=model_scale)
    return check_urdf(model)


def _ids(report) -> set[str]:
    return {f.check for f in report.findings}


def test_a_sane_model_reports_nothing(tmp_path: Path) -> None:
    report = _check(tmp_path, SANE)

    assert report.findings == []
    assert report.ok


def test_every_id_the_checks_emit_is_registered(tmp_path: Path) -> None:
    for finding in _check(tmp_path, SANE.replace('mass value="0.5"', 'mass value="0"')).findings:
        assert finding.check in CHECKS


def test_the_registry_and_the_documentation_list_the_same_ids() -> None:
    """checks.md is the contract; the registry is what CI actually runs."""
    documented = {
        line.split("|")[1].strip()
        for line in DOCS.read_text().splitlines()
        if line.startswith("| U")
        or line.startswith("| M")
        or line.startswith("| T")
        or line.startswith("| D")
    }

    assert documented == set(CHECKS)


# --------------------------------------------------------------------------- U001


def test_u001_fires_when_the_reach_is_implausible(tmp_path: Path) -> None:
    """One URDF unit of arm is one millimetre if the model is really in millimetres."""
    report = _check(tmp_path, SANE, model_scale=0.001)

    assert "U001" in _ids(report)
    assert not report.ok


def test_u001_warns_about_a_plausible_but_non_metre_model(tmp_path: Path) -> None:
    """The scale explains the numbers, but a physics engine still reads them as metres."""
    report = _check(tmp_path, SANE, model_scale=0.1)
    u001 = next(f for f in report.findings if f.check == "U001")

    assert u001.severity is Severity.WARN
    assert "decimetres" in u001.message


# --------------------------------------------------------------------------- U002-U004


def test_u002_catches_a_tensor_that_is_not_positive_definite(tmp_path: Path) -> None:
    body = SANE.replace(
        'ixx="0.00708" ixy="0" ixz="0" iyy="0.00708"',
        'ixx="-0.00708" ixy="0" ixz="0" iyy="0.00708"',
        1,
    )

    assert "U002" in _ids(_check(tmp_path, body))


def test_u003_catches_a_violated_triangle_inequality(tmp_path: Path) -> None:
    """I1 + I2 >= I3 holds for every rigid body; 1 + 1 < 10 does not."""
    body = f"""
      <link name="base">{inertial(1.0, (1.0, 1.0, 10.0))}{box("0.1 0.1 0.4")}</link>
    """
    report = _check(tmp_path, body)

    assert "U003" in _ids(report)


def test_u004_catches_a_tensor_computed_in_the_wrong_unit(tmp_path: Path) -> None:
    """A tensor worked out in millimetres and pasted into a metre model is 1e6 off."""
    body = SANE.replace(
        'ixx="0.00708" ixy="0" ixz="0" iyy="0.00708" iyz="0" izz="0.000833"',
        'ixx="7080" ixy="0" ixz="0" iyy="7080" iyz="0" izz="833"',
        1,
    )
    report = _check(tmp_path, body)

    assert "U004" in _ids(report)
    assert not report.ok


def test_u004_stays_quiet_for_a_hollow_part(tmp_path: Path) -> None:
    """A printed shell is not a solid box, so the check tolerates a factor of ten."""
    body = SANE.replace(
        'ixx="0.00708" ixy="0" ixz="0" iyy="0.00708"',
        'ixx="0.0212" ixy="0" ixz="0" iyy="0.0212"',
        1,
    )

    assert "U004" not in _ids(_check(tmp_path, body))


def test_u004_notices_a_tensor_declared_about_the_wrong_point(tmp_path: Path) -> None:
    """A common mistake: the right tensor, but <origin xyz> left at 0 0 0 while the
    part sits two metres away. About the link origin the real tensor is m*d^2 larger."""
    shape = (
        '<visual><origin xyz="0 0 2" rpy="0 0 0"/>'
        '<geometry><box size="0.1 0.1 0.4"/></geometry></visual>'
    )
    tensor = (
        '<mass value="1.0"/>'
        '<inertia ixx="0.0142" ixy="0" ixz="0" iyy="0.0142" iyz="0" izz="0.00167"/>'
    )

    def link(inertial_origin: str) -> str:
        return (
            f'<link name="a"><inertial><origin xyz="{inertial_origin}" rpy="0 0 0"/>'
            f"{tensor}</inertial>{shape}</link>"
        )

    assert "U004" not in _ids(_check(tmp_path, link("0 0 2")))
    assert "U004" in _ids(_check(tmp_path, link("0 0 0")))


def test_u004_needs_geometry_to_compare_against(tmp_path: Path) -> None:
    """A frame with a mass but no shape cannot be judged, and is not guessed at."""
    body = f'<link name="ghost">{inertial(1.0, (99.0, 99.0, 99.0))}</link>'

    assert "U004" not in _ids(_check(tmp_path, body))


# --------------------------------------------------------------------------- U005, U006


def test_u005_and_u006_catch_a_default_inertial_macro(tmp_path: Path) -> None:
    """Two differently shaped links, same tensor, same mass: nobody computed the physics."""
    placeholder = inertial(0.1, (1.0, 1.0, 1.0))
    body = f"""
      <link name="a">{placeholder}{box("0.1 0.1 0.4")}</link>
      <link name="b">{placeholder}{box("0.5 0.5 0.5")}</link>
      <joint name="j" type="fixed"><parent link="a"/><child link="b"/></joint>
    """
    report = _check(tmp_path, body)
    grouped = {f.check: f for f in report.findings if f.subjects}

    assert {"U005", "U006"} <= _ids(report)
    assert grouped["U005"].subjects == ["a", "b"]
    assert grouped["U006"].subjects == ["a", "b"]


def test_two_identical_parts_are_not_a_placeholder(tmp_path: Path) -> None:
    """A gripper's two fingers really do weigh the same. Crying wolf trains people
    to ignore the ID, so a shared value only counts when the parts differ in size."""
    same = inertial(0.1, (0.00142, 0.00142, 0.000167))
    body = f"""
      <link name="left">{same}{box("0.1 0.1 0.4")}</link>
      <link name="right">{same}{box("0.1 0.1 0.4")}</link>
      <joint name="j" type="fixed"><parent link="left"/><child link="right"/></joint>
    """
    report = _check(tmp_path, body)

    assert "U005" not in _ids(report)
    assert "U006" not in _ids(report)


def test_u006_fails_on_a_mass_of_zero(tmp_path: Path) -> None:
    body = f'<link name="a">{inertial(0.0, (1.0, 1.0, 1.0))}{box("0.1 0.1 0.4")}</link>'
    report = _check(tmp_path, body)
    u006 = next(f for f in report.findings if f.check == "U006")

    assert u006.severity is Severity.FAIL
    assert not report.ok


def test_u006_fails_when_a_link_with_geometry_has_no_inertial_at_all(tmp_path: Path) -> None:
    body = f'<link name="a">{box("0.1 0.1 0.4")}</link>'

    assert "U006" in _ids(_check(tmp_path, body))


def test_a_pure_frame_needs_no_inertial(tmp_path: Path) -> None:
    """`tip` and optical frames carry no mass by design; that is not a finding."""
    body = (
        SANE + '<link name="camera_frame"/><joint name="f" type="fixed">'
        '<parent link="tip"/><child link="camera_frame"/></joint>'
    )

    assert _check(tmp_path, body).findings == []


# --------------------------------------------------------------------------- D002

GROUNDED = SANE.replace(
    '<link name="base">',
    '<link name="world"/>'
    '<joint name="anchor" type="fixed">'
    '<parent link="world"/><child link="base"/><origin xyz="0 0 0" rpy="{rpy}"/></joint>'
    '<link name="base">',
)


def _with_mount(tmp_path: Path, rpy: str, mount: str):
    config = tmp_path / "mechlint.yaml"
    config.write_text(f"robot: {{ description: test.urdf }}\nscenario: {{ mount: {mount} }}\n")
    model = RobotModel.load(write_urdf(tmp_path, GROUNDED.format(rpy=rpy)))
    return check_urdf(model, MechlintConfig.from_yaml(config))


def test_d002_stays_quiet_when_the_config_and_the_model_agree(tmp_path: Path) -> None:
    assert "D002" not in _ids(_with_mount(tmp_path, "0 0 0", "table"))


def test_d002_reports_a_config_that_contradicts_the_model(tmp_path: Path) -> None:
    """The model wins -- it is the rotation the simulator runs -- so this warns and says
    which one was used, rather than failing or silently believing the file."""
    report = _with_mount(tmp_path, "0 0 0", "ceiling")
    d002 = next(f for f in report.findings if f.check == "D002")

    assert d002.severity is Severity.WARN
    assert "mechlint used the model" in d002.message
    assert report.ok


def test_d002_needs_a_world_frame_to_disagree_with(tmp_path: Path) -> None:
    """Without one there is nothing to compare against, and the config is just the answer."""
    config = tmp_path / "mechlint.yaml"
    config.write_text("robot: { description: test.urdf }\nscenario: { mount: ceiling }\n")
    model = RobotModel.load(write_urdf(tmp_path, SANE))

    assert "D002" not in _ids(check_urdf(model, MechlintConfig.from_yaml(config)))


def test_a_wall_mount_is_rejected_by_the_schema(tmp_path: Path) -> None:
    """One word cannot say which horizontal direction is down, so the name is gone."""
    config = tmp_path / "mechlint.yaml"
    config.write_text("robot: { description: test.urdf }\nscenario: { mount: wall }\n")

    with pytest.raises(ValidationError):
        MechlintConfig.from_yaml(config)


# --------------------------------------------------------------------------- reporting


def test_silence_is_not_a_pass(tmp_path: Path) -> None:
    """Checks that need a milestone that has not landed say so instead of nothing."""
    report = _check(tmp_path, SANE)

    assert set(report.skipped) == {"U007", "T001", "D001"}
    assert all(reason for reason in report.skipped.values())


def test_a_finding_carries_its_title_and_fix_from_the_registry(tmp_path: Path) -> None:
    finding = _check(tmp_path, SANE, model_scale=0.1).findings[0]

    assert finding.title == CHECKS["U001"].title
    assert finding.fix == CHECKS["U001"].fix


def test_warnings_alone_do_not_turn_ci_red(tmp_path: Path) -> None:
    report = _check(tmp_path, SANE, model_scale=0.1)

    assert _ids(report) == {"U001"}
    assert report.ok


# --------------------------------------------------------------------------- U007, D003


def _with_config(tmp_path: Path, yaml: str, body: str = SANE):
    path = tmp_path / "mechlint.yaml"
    path.write_text("robot: { description: test.urdf }\n" + yaml)
    model = RobotModel.load(write_urdf(tmp_path, body))
    return check_urdf(model, MechlintConfig.from_yaml(path))


def test_u007_catches_an_effort_limit_no_servo_can_reach(tmp_path: Path) -> None:
    """The SANE arm declares effort=5 N*m. An MG996R gives 1.079 N*m at 6 V, so a
    controller clamping at 5 never clamps at all."""
    report = _with_config(tmp_path, "actuators: { voltage: 6.0, joints: { j1: mg996r } }")
    (u007,) = [f for f in report.findings if f.check == "U007"]

    assert u007.severity is Severity.FAIL
    assert u007.subject == "j1"
    assert "4.63x" in u007.message
    assert "stall torque at 6 V" in u007.message


def test_u007_is_quiet_when_the_limit_is_honest(tmp_path: Path) -> None:
    body = SANE.replace('effort="5"', 'effort="1.0"')
    report = _with_config(tmp_path, "actuators: { voltage: 6.0, joints: { j1: mg996r } }", body)

    assert "U007" not in _ids(report)


def test_u007_says_it_did_not_run_rather_than_passing(tmp_path: Path) -> None:
    """Silence about a check nobody gave the inputs for reads as a pass. It is not one."""
    assert "U007" in _with_config(tmp_path, "actuators: { joints: { j1: null } }").skipped
    assert "U007" in _check(tmp_path, SANE).skipped


def test_d003_catches_a_link_renamed_on_one_side_only(tmp_path: Path) -> None:
    """`extra: forbid` catches a mistyped key. Nothing else catches a mistyped link name,
    and a servo on a link that does not exist is silently dropped -- which makes the robot
    lighter than it is, in the direction that looks safe."""
    report = _with_config(tmp_path, "components: { bass: [{ mass_g: 55, at: [0, 0, 0] }] }")
    (d003,) = [f for f in report.findings if f.check == "D003"]

    assert d003.severity is Severity.FAIL
    assert d003.subject == "components.bass"
    assert "no link called 'bass'" in d003.message


def test_d003_covers_materials_and_actuator_assignments(tmp_path: Path) -> None:
    report = _with_config(
        tmp_path,
        "materials: { links: { nope: { infill: 0.3 } } }\nactuators: { joints: { j9: mg996r } }",
    )
    subjects = {f.subject for f in report.findings if f.check == "D003"}

    assert subjects == {"materials.links.nope", "actuators.joints.j9"}


def test_d003_catches_a_servo_driving_a_joint_that_is_not_there(tmp_path: Path) -> None:
    report = _with_config(tmp_path, "components: { base: [{ actuator: mg996r, drives: j9 }] }")
    (d003,) = [f for f in report.findings if f.check == "D003"]

    assert "not a joint in the model" in d003.message


def test_a_config_that_matches_the_model_says_nothing(tmp_path: Path) -> None:
    report = _with_config(
        tmp_path,
        "materials: { links: { base: { infill: 0.3 } } }\n"
        "components: { base: [{ actuator: mg996r, drives: j1 }] }\n"
        "actuators: { voltage: 6.0, joints: { j1: null } }",
    )

    assert "D003" not in _ids(report)

"""Every check, on a model built to fail exactly one of them.

The DAST-1 fixture proves the checks fire on a real robot; these prove they
fire for the reason they claim, and stay quiet otherwise.
"""

from __future__ import annotations

from pathlib import Path

from mechlint.core.checks import CHECKS, Severity, check_urdf
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

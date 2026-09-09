"""M1 on the real robot: what `urdf-check` and `inertia` say about DAST-1 today.

The plan names this milestone done when, on this fixture, mechlint reports U001
(the model is in decimetres), U005/U006 (the `default_inertial` macro), M001
(`arm_1.stl` and `base.stl` are not watertight), and writes an `inertials.xacro`
for every link it can compute. These tests are that sentence, executable.

The numbers here are pinned. When a mesh changes, re-record them deliberately --
see tests/fixtures/dast1/PROVENANCE.md -- rather than loosening the tolerance.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from typer.testing import CliRunner

from mechlint.cli.main import app
from mechlint.core.checks import Severity, check_urdf
from mechlint.core.config import MechlintConfig
from mechlint.core.inertia import compute_inertia, inertials_xacro
from mechlint.core.urdf import PackageResolver, RobotModel

runner = CliRunner()

#: PLA at 1240 kg/m^3 and the 30 % infill in the fixture's mechlint.yaml.
#: base_link is 210 g because it was weighed; the rest are estimates.
#: arm_1 and base come from a convex hull, so they are upper bounds (M001).
MASS_BASELINE_G = {
    "base_link": 210.000,
    "rotary_link": 11.908,
    "arm_1_link": 123.139,
    "arm_2_link": 38.887,
    "arm_3_link": 35.982,
    "gripper_link": 43.278,
    "kinect_link": 291.648,
}

LINKS_WITHOUT_GEOMETRY = {"world", "wrist_link", "tip", "kinect_rgb_optical_frame"}


@pytest.fixture(scope="module")
def config(dast1_dir: Path) -> MechlintConfig:
    return MechlintConfig.from_yaml(dast1_dir / "mechlint.yaml")


@pytest.fixture(scope="module")
def model(config: MechlintConfig) -> RobotModel:
    """Loaded the way a user would: through the config, from the xacro, without ROS."""
    resolver = PackageResolver(
        {name: config.resolve(path) for name, path in config.robot.package_paths.items()}
    )
    return RobotModel.load(
        config.description_path,
        xacro_args=config.robot.xacro_args,
        resolver=resolver,
        model_scale=float(config.robot.model_scale),
    )


# --------------------------------------------------------------------------- urdf-check


def test_the_xacro_expands_without_a_sourced_ros_workspace(model: RobotModel) -> None:
    """The whole ROS-free-core promise, on a description written for ROS."""
    assert model.name == "dast_1"
    assert model.urdf.actuated_joint_names == [f"joint_{i}" for i in range(1, 7)]


def test_the_milestone_checks_all_fire(model: RobotModel, config: MechlintConfig) -> None:
    report = check_urdf(model, config)
    found = {f.check for f in report.findings}

    assert {"U001", "U004", "U005", "U006", "M001"} <= found
    assert not report.ok


def test_u001_names_the_decimetre_model(model: RobotModel, config: MechlintConfig) -> None:
    """0.759 m of reach is plausible; 7.59 URDF units of it is not metres."""
    u001 = next(f for f in check_urdf(model, config).findings if f.check == "U001")

    assert u001.severity is Severity.WARN
    assert "decimetres" in u001.message
    assert "0.759 m" in u001.message
    assert "millimetres" in u001.message  # the meshes, which are scaled by 0.01


def test_u005_and_u006_catch_the_default_inertial_macro(
    model: RobotModel, config: MechlintConfig
) -> None:
    findings = check_urdf(model, config).findings
    u005 = next(f for f in findings if f.check == "U005")
    masses = [f for f in findings if f.check == "U006"]

    assert len(u005.subjects) == 8
    assert "[1, 1, 1]" in u005.message
    assert {"arm_1_link", "arm_2_link", "arm_3_link"} <= {s for f in masses for s in f.subjects}


def test_m001_names_the_two_open_meshes(model: RobotModel, config: MechlintConfig) -> None:
    open_meshes = {
        f.subject.split(": ")[1] for f in check_urdf(model, config).findings if f.check == "M001"
    }

    assert open_meshes == {"arm_1.stl", "base.stl"}


def test_u007_is_reported_as_not_run_rather_than_passing(
    model: RobotModel, config: MechlintConfig
) -> None:
    """DAST-1 declares effort=20 N*m on every joint against an MG996R's ~1.1 N*m.
    That is exactly U007, and silence about it would read as a pass."""
    assert "U007" in check_urdf(model, config).skipped


# --------------------------------------------------------------------------- inertia


@pytest.mark.parametrize("link", sorted(MASS_BASELINE_G))
def test_the_computed_masses_hold(model: RobotModel, config: MechlintConfig, link: str) -> None:
    computed = {entry.link: entry for entry in compute_inertia(model, config).links}

    assert computed[link].mass_kg * 1e3 == pytest.approx(MASS_BASELINE_G[link], abs=1e-3)


def test_the_measured_mass_wins_over_the_estimate(
    model: RobotModel, config: MechlintConfig
) -> None:
    base = next(link for link in compute_inertia(model, config).links if link.link == "base_link")

    assert base.mass_source == "measured"
    assert base.mass_kg == pytest.approx(0.210)
    assert base.convex_hull_fallback  # base.stl is open, so the shape is still a hull


def test_the_kinect_on_its_stick_is_not_part_of_what_the_joints_hold(
    model: RobotModel, config: MechlintConfig
) -> None:
    """The Kinect is eye-to-hand: mounted on a stick beside the robot, 0.9 m behind the
    base, so no actuator carries it. Nothing in mechlint.yaml says so -- it falls out of
    the chain, because every joint between it and the root is fixed. Same for base_link."""
    report = compute_inertia(model, config)

    assert set(report.uncarried_links) == {"base_link", "kinect_link"}
    assert report.carried_mass_kg == pytest.approx(0.2632, abs=1e-4)
    assert report.total_mass_kg == pytest.approx(0.7648, abs=1e-4)


def test_the_two_totals_add_up(model: RobotModel, config: MechlintConfig) -> None:
    """A reader should never have to subtract two numbers out of a report themselves."""
    report = compute_inertia(model, config)

    assert report.carried_mass_kg + report.uncarried_mass_kg == pytest.approx(report.total_mass_kg)


def test_the_table_shows_both_totals(model: RobotModel, config: MechlintConfig) -> None:
    """One figure invites reading a tripod-mounted camera as part of the arm."""
    from mechlint.core.report import render_inertia

    table = render_inertia(compute_inertia(model, config))

    assert "carried by a joint           263.2 g" in table
    assert "not carried                  501.6 g   base_link, kinect_link" in table


def test_the_five_servos_are_reported_missing_not_silently_omitted(
    model: RobotModel, config: MechlintConfig
) -> None:
    pending = compute_inertia(model, config).pending_components

    assert len(pending) == 5
    assert {p.label for p in pending} == {"mg996r"}


def test_an_inertial_is_written_for_every_link_with_geometry(
    model: RobotModel, config: MechlintConfig
) -> None:
    report = compute_inertia(model, config)
    written = {link.link for link in report.writable_links}
    text = inertials_xacro(report)

    assert written == set(MASS_BASELINE_G)
    assert written | LINKS_WITHOUT_GEOMETRY == set(model.link_names)
    for link in written:
        assert f"mechlint_inertial_{link}" in text


def test_the_generated_file_says_it_is_in_decimetres(
    model: RobotModel, config: MechlintConfig
) -> None:
    """The one number a reader of that file could get catastrophically wrong."""
    text = inertials_xacro(compute_inertia(model, config))

    assert "1 unit = 0.1 m" in text
    assert "DO NOT EDIT" in text


def test_the_generated_inertials_clear_the_tensor_checks(
    model: RobotModel, config: MechlintConfig
) -> None:
    """The round trip that matters: the numbers in the file mechlint writes, put back
    into the model, no longer trip U002-U006. Reading them out of the emitted XML
    rather than off the report is deliberate -- it is the unit conversion in that
    file, not the SI value behind it, that a simulator will actually load."""
    from xml.dom.minidom import parseString

    import numpy as np

    document = parseString(inertials_xacro(compute_inertia(model, config)))
    for macro in document.getElementsByTagName("xacro:macro"):
        link = macro.getAttribute("name").removeprefix("mechlint_inertial_")
        (block,) = macro.getElementsByTagName("inertia")
        (origin,) = macro.getElementsByTagName("origin")
        (mass,) = macro.getElementsByTagName("mass")

        declared = model.declared_inertial(link)
        declared.mass = float(mass.getAttribute("value"))
        declared.origin = np.eye(4)
        declared.origin[:3, 3] = [float(v) for v in origin.getAttribute("xyz").split()]
        names = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
        values = {name: float(block.getAttribute(name)) for name in names}
        declared.inertia = np.array(
            [
                [values["ixx"], values["ixy"], values["ixz"]],
                [values["ixy"], values["iyy"], values["iyz"]],
                [values["ixz"], values["iyz"], values["izz"]],
            ]
        )

    remaining = {f.check for f in check_urdf(model, config).findings}

    assert remaining == {"U001", "M001"}  # the model is still in decimetres, meshes still open


# --------------------------------------------------------------------------- the CLI


def test_the_cli_finds_its_own_config(dast1_dir: Path, tmp_path: Path) -> None:
    """`mechlint check` in the robot's directory needs no arguments at all."""
    result = runner.invoke(app, ["check", "-c", str(dast1_dir / "mechlint.yaml"), "-f", "json"])
    payload = json.loads(result.output)

    assert result.exit_code == 1
    assert payload["robot"] == "dast_1"
    assert payload["ok"] is False


def test_the_cli_writes_inertials_for_the_fixture(dast1_dir: Path, tmp_path: Path) -> None:
    result = runner.invoke(
        app,
        ["inertia", "-c", str(dast1_dir / "mechlint.yaml"), "-w", str(tmp_path)],
    )

    assert result.exit_code == 0
    text = (tmp_path / "inertials.xacro").read_text()
    for link in MASS_BASELINE_G:
        assert f'name="mechlint_inertial_{link}"' in text

"""Composite mass properties, against tensors that can be worked out by hand."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from mechlint.core.config import MechlintConfig
from mechlint.core.inertia import (
    GENERATED_MARKER,
    WriteRefused,
    compute_inertia,
    inertials_xacro,
    write_inertials,
)
from mechlint.core.urdf import RobotModel
from tests.unit.urdf_builder import box, inertial, write_urdf

#: A 1 kg PLA-density box would be an odd thing to hand-check, so the tests below
#: pick geometry whose analytic tensor is a one-line formula.
ARM = f"""
  <link name="base">{inertial(0.1, (1, 1, 1))}{box("0.2 0.2 0.2")}</link>
  <link name="tool">{inertial(0.1, (1, 1, 1))}{box("0.1 0.1 0.4")}</link>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="tool"/>
    <origin xyz="0 0 0.5"/><axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="5" velocity="1"/>
  </joint>
"""


def _model(tmp_path: Path, body: str = ARM, scale: float = 1.0) -> RobotModel:
    return RobotModel.load(write_urdf(tmp_path, body), model_scale=scale)


def _links(report: object) -> dict[str, object]:
    return {link.link: link for link in report.links}  # type: ignore[attr-defined]


def _config(tmp_path: Path, yaml: str) -> MechlintConfig:
    path = tmp_path / "mechlint.yaml"
    path.write_text("robot: { description: test.urdf }\n" + yaml)
    return MechlintConfig.from_yaml(path)


def test_a_solid_box_gets_its_analytic_tensor(tmp_path: Path) -> None:
    """0.2 m cube of PLA at full infill: 1240 * 0.008 = 9.92 kg, I = m/6 * a^2."""
    report = compute_inertia(_model(tmp_path), _config(tmp_path, "materials: {default: {}}"))
    link = next(link for link in report.links if link.link == "base")

    assert link.mass_kg == pytest.approx(1240 * 0.2**3)
    assert np.diag(link.inertia_kg_m2) == pytest.approx([link.mass_kg * 0.2**2 / 6] * 3)
    assert link.mass_source == "estimated"


def test_infill_scales_the_mass_and_the_tensor_together(tmp_path: Path) -> None:
    """Infill is a slicer setting, so it lands on the density, not on the geometry."""
    solid = compute_inertia(_model(tmp_path), _config(tmp_path, "materials: {default: {}}"))
    third = compute_inertia(
        _model(tmp_path), _config(tmp_path, "materials: {default: {infill: 0.3}}")
    )

    assert third.links[0].mass_kg == pytest.approx(solid.links[0].mass_kg * 0.3)
    assert np.asarray(third.links[0].inertia_kg_m2) == pytest.approx(
        np.asarray(solid.links[0].inertia_kg_m2) * 0.3
    )


def test_a_measured_mass_replaces_the_estimate_and_rescales_the_tensor(tmp_path: Path) -> None:
    report = compute_inertia(
        _model(tmp_path),
        _config(tmp_path, "materials: {links: {base: {measured_mass_g: 500}}}"),
    )
    link = next(link for link in report.links if link.link == "base")

    assert link.mass_kg == pytest.approx(0.5)
    assert link.mass_source == "measured"
    assert np.diag(link.inertia_kg_m2) == pytest.approx([0.5 * 0.2**2 / 6] * 3)


def test_a_component_is_a_point_mass_that_moves_the_centre(tmp_path: Path) -> None:
    """100 g cube centred at the origin plus 100 g at z = 0.1 sits at z = 0.05,
    and the two halves each contribute m*d^2 about that new centre."""
    report = compute_inertia(
        _model(tmp_path),
        _config(
            tmp_path,
            "materials: {links: {base: {measured_mass_g: 100}}}\n"
            "components: {base: [{mass_g: 100, at: [0, 0, 0.1], note: battery}]}\n",
        ),
    )
    link = next(link for link in report.links if link.link == "base")

    assert link.mass_kg == pytest.approx(0.2)
    assert link.com_m == pytest.approx((0.0, 0.0, 0.05))
    assert link.inertia_kg_m2[2][2] == pytest.approx(0.1 * 0.2**2 / 6)
    assert link.inertia_kg_m2[0][0] == pytest.approx(0.1 * 0.2**2 / 6 + 2 * 0.1 * 0.05**2)


def test_a_component_that_only_names_a_joint_sits_at_that_joint(tmp_path: Path) -> None:
    """A servo is physically at the joint it turns, which is where the URDF puts it."""
    report = compute_inertia(
        _model(tmp_path),
        _config(tmp_path, "components: {base: [{mass_g: 55, drives: j1, actuator: mg996r}]}"),
    )
    component = next(link for link in report.links if link.link == "base").components[0]

    assert component.at_m == pytest.approx((0.0, 0.0, 0.5))


def test_an_actuator_gets_its_mass_from_the_database(tmp_path: Path) -> None:
    """The whole point of `actuator:` with no `mass_g`: the database knows what it weighs."""
    report = compute_inertia(
        _model(tmp_path), _config(tmp_path, "components: {base: [{actuator: mg996r, drives: j1}]}")
    )
    (component,) = next(link for link in report.links if link.link == "base").components

    assert report.pending_components == []
    assert component.mass_kg == pytest.approx(0.055)
    assert "MG996R" in component.source


def test_an_unknown_actuator_is_reported_not_dropped(tmp_path: Path) -> None:
    """A servo silently missing is the difference between right and comfortably wrong."""
    report = compute_inertia(
        _model(tmp_path), _config(tmp_path, "components: {base: [{actuator: nope, drives: j1}]}")
    )
    (pending,) = report.pending_components

    assert pending.ok is False
    assert "unknown actuator" in pending.error
    assert "mass_g" in pending.hint


def test_a_link_with_nothing_to_compute_from_keeps_its_urdf_mass(tmp_path: Path) -> None:
    body = ARM + '<link name="frame">' + inertial(0.01, (1, 1, 1)) + "</link>"
    report = compute_inertia(_model(tmp_path, body))
    link = next(link for link in report.links if link.link == "frame")

    assert link.mass_source == "urdf"
    assert link.mass_kg == pytest.approx(0.01)
    assert not link.writable


def test_the_base_is_not_carried_by_anything(tmp_path: Path) -> None:
    """No joint holds the base up, so its mass belongs in no torque sum. The two
    totals are split for that reason, and they still add up."""
    report = compute_inertia(_model(tmp_path))
    carried = {link.link for link in report.links if link.carried}

    assert carried == {"tool"}
    assert report.carried_mass_kg + report.uncarried_mass_kg == pytest.approx(report.total_mass_kg)
    assert report.uncarried_links == ["base"]


def test_a_link_bolted_to_the_world_beside_the_robot_is_not_carried(tmp_path: Path) -> None:
    """A tripod-mounted camera is in the model because its frame matters, not because
    an actuator holds it. Fixed joints all the way to the root is the whole test --
    nothing has to be declared in mechlint.yaml."""
    body = (
        ARM
        + f"""
      <link name="camera">{inertial(0.5, (1, 1, 1))}{box("0.1 0.1 0.1")}</link>
      <joint name="tripod" type="fixed">
        <parent link="base"/><child link="camera"/><origin xyz="0 0.9 0"/>
      </joint>
    """
    )
    report = compute_inertia(_model(tmp_path, body))
    camera = next(link for link in report.links if link.link == "camera")

    assert camera.carried is False
    assert camera.mass_kg > 0  # it is still weighed, still checked, still written
    assert set(report.uncarried_links) == {"base", "camera"}


def test_the_model_scale_is_applied_once(tmp_path: Path) -> None:
    """The same robot described in decimetres weighs the same as one in metres,
    and its tensor comes out identical -- the scale is applied once, at the boundary."""
    body = '<link name="a">' + box("0.2 0.2 0.2") + "</link>"
    metres = compute_inertia(_model(tmp_path, body))
    decimetres = compute_inertia(_model(tmp_path, body.replace("0.2 0.2 0.2", "2 2 2"), scale=0.1))

    assert decimetres.total_mass_kg == pytest.approx(metres.total_mass_kg)
    assert np.asarray(decimetres.links[0].inertia_kg_m2) == pytest.approx(
        np.asarray(metres.links[0].inertia_kg_m2)
    )


# --------------------------------------------------------------------------- emit


def test_the_generated_xacro_is_in_urdf_units(tmp_path: Path) -> None:
    """A decimetre model stays a decimetre model: 1 kg*m^2 is 100 kg*dm^2."""
    report = compute_inertia(_model(tmp_path, ARM.replace("0.2 0.2 0.2", "2 2 2"), scale=0.1))
    link = next(link for link in report.links if link.link == "base")
    text = inertials_xacro(report)

    written = float(text.split('ixx="')[1].split('"')[0])
    assert written == pytest.approx(link.inertia_kg_m2[0][0] / 0.1**2, rel=1e-5)
    assert "1 unit = 0.1 m" in text


def test_the_generated_xacro_parses_as_xacro(tmp_path: Path) -> None:
    import xacro

    report = compute_inertia(_model(tmp_path))
    path = tmp_path / "inertials.xacro"
    path.write_text(inertials_xacro(report))

    document = xacro.process_file(str(path))

    assert document.documentElement.tagName == "robot"
    assert "mechlint_inertial_base" in document.toxml()


def test_only_computed_links_get_a_macro(tmp_path: Path) -> None:
    """mechlint does not invent a tensor for a link it has no geometry for."""
    body = ARM + '<link name="frame">' + inertial(0.01, (1, 1, 1)) + "</link>"
    text = inertials_xacro(compute_inertia(_model(tmp_path, body)))

    assert "mechlint_inertial_base" in text
    assert "mechlint_inertial_frame" not in text


def test_the_generated_inertials_pass_the_checks_that_the_placeholders_failed(
    tmp_path: Path,
) -> None:
    """The round trip that matters: what `inertia` writes, `urdf-check` accepts."""
    from mechlint.core.checks import check_urdf

    model = _model(tmp_path)
    before = {f.check for f in check_urdf(model).findings}
    assert {"U004", "U005", "U006"} <= before

    for link in compute_inertia(model).writable_links:
        declared = model.declared_inertial(link.link)
        declared.mass = link.mass_kg
        declared.inertia = np.asarray(link.inertia_kg_m2)
        declared.origin = np.eye(4)
        declared.origin[:3, 3] = link.com_m

    after = {f.check for f in check_urdf(model).findings}
    assert after == set()


# --------------------------------------------------------------------------- overrides


def test_a_link_mass_override_keeps_the_shape_and_scales_the_tensor(tmp_path: Path) -> None:
    """Suppose it came out at 200 g: a different mass, not a different part, so the
    centre of mass stays where the geometry put it and the tensor follows the mass."""
    model = _model(tmp_path)
    before = _links(compute_inertia(model))["tool"]
    after = _links(compute_inertia(model, link_masses_g={"tool": 200.0}))["tool"]

    assert after.mass_kg == pytest.approx(0.2)
    assert after.mass_source == "override"
    assert after.com_m == before.com_m
    ratio = 0.2 / before.mass_kg
    assert np.allclose(np.asarray(after.inertia_kg_m2), np.asarray(before.inertia_kg_m2) * ratio)


def test_an_overridden_link_is_never_written(tmp_path: Path) -> None:
    """A hypothetical that reaches the robot description is worse than no answer."""
    report = compute_inertia(_model(tmp_path), link_masses_g={"tool": 200.0})

    assert [link.link for link in report.writable_links] == ["base"]
    assert report.overrides == {"link_mass_g": {"tool": 200.0}}
    with pytest.raises(WriteRefused, match="hypothetical"):
        write_inertials(report, tmp_path / "inertials.xacro")


def test_a_mass_override_says_what_it_replaced(tmp_path: Path) -> None:
    report = compute_inertia(_model(tmp_path), link_masses_g={"tool": 200.0})
    notes = " ".join(_links(report)["tool"].notes)

    assert "200 g instead of" in notes


def test_an_override_for_a_link_that_does_not_exist_is_refused(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="no link called 'tooll'"):
        compute_inertia(_model(tmp_path), link_masses_g={"tooll": 200.0})


def test_a_report_with_no_overrides_says_so(tmp_path: Path) -> None:
    assert compute_inertia(_model(tmp_path)).overrides == {}


# --------------------------------------------------------------------------- writing


def test_write_inertials_creates_the_file_and_reports_what_it_skipped(tmp_path: Path) -> None:
    report = compute_inertia(_model(tmp_path))
    result = write_inertials(report, tmp_path / "gen" / "inertials.xacro")

    assert result.created is True
    assert result.unchanged is False
    assert result.links == ["base", "tool"]
    assert result.bytes_written == len((tmp_path / "gen" / "inertials.xacro").read_text().encode())


def test_a_directory_target_becomes_the_default_filename(tmp_path: Path) -> None:
    result = write_inertials(compute_inertia(_model(tmp_path)), tmp_path)

    assert result.path == tmp_path / "inertials.xacro"
    assert result.include == '<xacro:include filename="inertials.xacro"/>'


def test_rewriting_the_same_model_is_a_no_op(tmp_path: Path) -> None:
    report = compute_inertia(_model(tmp_path))
    write_inertials(report, tmp_path)

    assert write_inertials(report, tmp_path).unchanged is True


def test_write_refuses_a_file_it_did_not_generate_unless_forced(tmp_path: Path) -> None:
    target = tmp_path / "handwritten.xacro"
    target.write_text("<robot><!-- mine --></robot>")
    report = compute_inertia(_model(tmp_path))

    with pytest.raises(WriteRefused, match="not generated by mechlint"):
        write_inertials(report, target)
    assert target.read_text() == "<robot><!-- mine --></robot>"

    assert write_inertials(report, target, force=True).created is False
    assert GENERATED_MARKER in target.read_text()
